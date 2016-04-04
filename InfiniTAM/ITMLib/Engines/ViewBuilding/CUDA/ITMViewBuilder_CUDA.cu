// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMViewBuilder_CUDA.h"

#include "../Shared/ITMViewBuilder_Shared.h"
#include "../../../../ORUtils/CUDADefines.h"
#include "../../../../ORUtils/MemoryBlock.h"

#include "../../../../ORUtils/FileUtils.h"

using namespace ITMLib;
using namespace ORUtils;

ITMViewBuilder_CUDA::ITMViewBuilder_CUDA(const ITMRGBDCalib *calib, const Vector2i &paddingSize) :ITMViewBuilder(calib, paddingSize) { }
ITMViewBuilder_CUDA::~ITMViewBuilder_CUDA(void) { }

//---------------------------------------------------------------------------
//
// kernel function declaration 
//
//---------------------------------------------------------------------------


__global__ void convertDisparityToDepth_device(float *depth_out, const short *depth_in, Vector2f disparityCalibParams, float fx_depth, Vector2i imgSize);
__global__ void convertDepthAffineToFloat_device(float *d_out, const short *d_in, Vector2i imgSize, Vector2f depthCalibParams);
__global__ void filterDepth_device(float *imageData_out, const float *imageData_in, Vector2i imgDims);
__global__ void ComputeNormalAndWeight_device(const float* depth_in, Vector4f* normal_out, float *sigmaL_out, Vector2i imgDims, Vector4f intrinsic);

//---------------------------------------------------------------------------
//
// host methods
//
//---------------------------------------------------------------------------

void ITMViewBuilder_CUDA::UpdateView(ITMView **view_ptr, ITMUChar4Image *in_rgbImage, ITMShortImage *in_rawDepthImage, bool useBilateralFilter, bool modelSensorNoise)
{
	if (*view_ptr == NULL)
	{
		Vector2i newDims_depth, newDims_rgb;

		newDims_depth = 2 * paddingSize + in_rawDepthImage->noDims;
		newDims_rgb = 2 * paddingSize + in_rgbImage->noDims;

		if (this->rawDepthImage != NULL) delete this->rawDepthImage;
		this->rawDepthImage = new ITMShortImage(newDims_depth, true, true);

		if (this->rgbImage != NULL) delete this->rgbImage;
		this->rgbImage = new ITMUChar4Image(newDims_rgb, true, true);

		*view_ptr = new ITMView(calib, rgbImage->noDims, rawDepthImage->noDims, true);
		if (this->shortImage != NULL) delete this->shortImage;
		this->shortImage = new ITMShortImage(rawDepthImage->noDims, true, true);
		if (this->floatImage != NULL) delete this->floatImage;
		this->floatImage = new ITMFloatImage(rawDepthImage->noDims, true, true);

		if (modelSensorNoise)
		{
			(*view_ptr)->depthNormals = new ITMFloat4Image(rawDepthImage->noDims, true, true);
			(*view_ptr)->depthUncertainty = new ITMFloatImage(rawDepthImage->noDims, true, true);
		}
	}

	ITMView *view = *view_ptr;

	this->PadImage(this->rawDepthImage, in_rawDepthImage, paddingSize);
	this->PadImage(this->rgbImage, in_rgbImage, paddingSize);

	view->rgb->SetFrom(rgbImage, MemoryBlock<Vector4u>::CPU_TO_CUDA);
	this->shortImage->SetFrom(rawDepthImage, MemoryBlock<short>::CPU_TO_CUDA);

	switch (view->calib->disparityCalib.type)
	{
	case ITMDisparityCalib::TRAFO_KINECT:
		this->ConvertDisparityToDepth(view->depth, this->shortImage, &(view->calib->intrinsics_d), view->calib->disparityCalib.params);
		break;
	case ITMDisparityCalib::TRAFO_AFFINE:
		this->ConvertDepthAffineToFloat(view->depth, this->shortImage, view->calib->disparityCalib.params);
		break;
	default:
		break;
	}

	// set confidence --> should move to GPU
	float *confidence = view->depthConfidence->GetData(MEMORYDEVICE_CPU);
	const float halfW = rawDepthImage->noDims.x / 2.0f, halfH = rawDepthImage->noDims.y / 2.0f;
	const float sigma = 0.6f;
	for (int y = 0; y < rawDepthImage->noDims.y; y++) for (int x = 0; x < rawDepthImage->noDims.x; x++)
	{
		//const float dx = abs(x - halfW), dy = abs(y - halfH);
		//const float gamma = sqrtf((dx * dx + dy * dy) / (halfW * halfW + halfH * halfH));

		// Calculate and return the confidence value itself.
		confidence[x + y * rawDepthImage->noDims.x] = 1.0f;// expf(-gamma*gamma) / (2 * sigma*sigma);
	}
	view->depthConfidence->UpdateDeviceFromHost();

	if (useBilateralFilter)
	{
		//5 steps of bilateral filtering
		this->DepthFiltering(this->floatImage, view->depth);
		this->DepthFiltering(view->depth, this->floatImage);
		this->DepthFiltering(this->floatImage, view->depth);
		this->DepthFiltering(view->depth, this->floatImage);
		this->DepthFiltering(this->floatImage, view->depth);

		view->depth->SetFrom(this->floatImage, MemoryBlock<float>::CUDA_TO_CUDA);
	}

	if (modelSensorNoise)
	{
		this->ComputeNormalAndWeights(view->depthNormals, view->depthUncertainty, view->depth, view->calib->intrinsics_d.projectionParamsSimple.all);
	}

	//view->depthUncertainty->UpdateHostFromDevice();
	//view->depth->UpdateHostFromDevice();
	//WriteToBIN(view->depth->GetData(MEMORYDEVICE_CPU), view->depth->dataSize, "c:/temp/data.bin");
}

//void ITMViewBuilder_CUDA::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage)
//{
//	if (*view_ptr == NULL)
//		*view_ptr = new ITMView(calib, rgbImage->noDims, depthImage->noDims, true);
//
//	ITMView *view = *view_ptr;
//
//	view->rgb->UpdateDeviceFromHost();
//	view->depth->UpdateDeviceFromHost();
//}

void ITMViewBuilder_CUDA::UpdateView(ITMView **view_ptr, ITMUChar4Image *in_rgbImage, ITMShortImage *in_rawDepthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement, bool modelSensorNoise)
{
	if (*view_ptr == NULL) 
	{
		Vector2i newDims_depth, newDims_rgb;

		newDims_depth = 2 * paddingSize + in_rawDepthImage->noDims;
		newDims_rgb = 2 * paddingSize + in_rgbImage->noDims;

		if (this->rawDepthImage != NULL) delete this->rawDepthImage;
		this->rawDepthImage = new ITMShortImage(newDims_depth, true, true);

		if (this->rgbImage != NULL) delete this->rgbImage;
		this->rgbImage = new ITMUChar4Image(newDims_rgb, true, true);

		*view_ptr = new ITMViewIMU(calib, rgbImage->noDims, depthImage->noDims, true);
		if (this->shortImage != NULL) delete this->shortImage;
		this->shortImage = new ITMShortImage(depthImage->noDims, true, true);
		if (this->floatImage != NULL) delete this->floatImage;
		this->floatImage = new ITMFloatImage(depthImage->noDims, true, true);

		if (modelSensorNoise)
		{
			(*view_ptr)->depthNormals = new ITMFloat4Image(depthImage->noDims, true, true);
			(*view_ptr)->depthUncertainty = new ITMFloatImage(depthImage->noDims, true, true);
		}
	}

	ITMViewIMU* imuView = (ITMViewIMU*)(*view_ptr);
	imuView->imu->SetFrom(imuMeasurement);

	this->UpdateView(view_ptr, rgbImage, rawDepthImage, useBilateralFilter, modelSensorNoise);
}

void ITMViewBuilder_CUDA::ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const ITMIntrinsics *depthIntrinsics,
	Vector2f disparityCalibParams)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CUDA);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CUDA);

	float fx_depth = depthIntrinsics->projectionParamsSimple.fx;

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)blockSize.x), (int)ceil((float)imgSize.y / (float)blockSize.y));

	convertDisparityToDepth_device << <gridSize, blockSize >> >(d_out, d_in, disparityCalibParams, fx_depth, imgSize);
	ORcudaKernelCheck;
}

void ITMViewBuilder_CUDA::ConvertDepthAffineToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in, Vector2f depthCalibParams)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CUDA);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)blockSize.x), (int)ceil((float)imgSize.y / (float)blockSize.y));

	convertDepthAffineToFloat_device << <gridSize, blockSize >> >(d_out, d_in, imgSize, depthCalibParams);
	ORcudaKernelCheck;
}

void ITMViewBuilder_CUDA::DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in)
{
	Vector2i imgDims = image_in->noDims;

	const float *imageData_in = image_in->GetData(MEMORYDEVICE_CUDA);
	float *imageData_out = image_out->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgDims.x / (float)blockSize.x), (int)ceil((float)imgDims.y / (float)blockSize.y));

	filterDepth_device << <gridSize, blockSize >> >(imageData_out, imageData_in, imgDims);
	ORcudaKernelCheck;
}

void ITMViewBuilder_CUDA::ComputeNormalAndWeights(ITMFloat4Image *normal_out, ITMFloatImage *sigmaZ_out, const ITMFloatImage *depth_in, Vector4f intrinsic)
{
	Vector2i imgDims = depth_in->noDims;

	const float *depthData_in = depth_in->GetData(MEMORYDEVICE_CUDA);

	float *sigmaZData_out = sigmaZ_out->GetData(MEMORYDEVICE_CUDA);
	Vector4f *normalData_out = normal_out->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgDims.x / (float)blockSize.x), (int)ceil((float)imgDims.y / (float)blockSize.y));

	ComputeNormalAndWeight_device << <gridSize, blockSize >> >(depthData_in, normalData_out, sigmaZData_out, imgDims, intrinsic);
}

//---------------------------------------------------------------------------
//
// kernel function implementation
//
//---------------------------------------------------------------------------

__global__ void convertDisparityToDepth_device(float *d_out, const short *d_in, Vector2f disparityCalibParams, float fx_depth, Vector2i imgSize)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x;
	int y = threadIdx.y + blockIdx.y * blockDim.y;

	if ((x >= imgSize.x) || (y >= imgSize.y)) return;

	convertDisparityToDepth(d_out, x, y, d_in, disparityCalibParams, fx_depth, imgSize);
}

__global__ void convertDepthAffineToFloat_device(float *d_out, const short *d_in, Vector2i imgSize, Vector2f depthCalibParams)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x;
	int y = threadIdx.y + blockIdx.y * blockDim.y;

	if ((x >= imgSize.x) || (y >= imgSize.y)) return;

	convertDepthAffineToFloat(d_out, x, y, d_in, imgSize, depthCalibParams);
}

__global__ void filterDepth_device(float *imageData_out, const float *imageData_in, Vector2i imgDims)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	imageData_out[x + y * imgDims.x] = imageData_in[x + y * imgDims.x];

	if (x < 2 || x > imgDims.x - 2 || y < 2 || y > imgDims.y - 2)
		return;

	filterDepth(imageData_out, imageData_in, x, y, imgDims);
}

__global__ void ComputeNormalAndWeight_device(const float* depth_in, Vector4f* normal_out, float *sigmaZ_out, Vector2i imgDims, Vector4f intrinsic)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;
	int idx = x + y * imgDims.x;

	if (x < 2 || x > imgDims.x - 2 || y < 2 || y > imgDims.y - 2)
	{
		normal_out[idx].w = -1.0f;
		sigmaZ_out[idx] = -1;
		return;
	}
	else
	{
		computeNormalAndWeight(depth_in, normal_out, sigmaZ_out, x, y, imgDims, intrinsic);
	}
}

