// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMViewBuilder_CUDA.h"
#include "../../../../ORUtils/CUDADefines.h"

#include "../../DeviceAgnostic/ITMViewBuilder.h"
#include "../../../../ORUtils/MemoryBlock.h"

using namespace ITMLib::Engine;
using namespace ORUtils;

ITMViewBuilder_CUDA::ITMViewBuilder_CUDA(const ITMRGBDCalib *calib):ITMViewBuilder(calib) { }
ITMViewBuilder_CUDA::~ITMViewBuilder_CUDA(void) { }

__global__ void convertDisparityToDepth_device(float *depth_out, const short *depth_in, Vector2f disparityCalibParams, float fx_depth, Vector2i imgSize);
__global__ void convertDepthMMToFloat_device(float *d_out, const short *d_in, Vector2i imgSize);

// host methods

void ITMViewBuilder_CUDA::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
	if (*view_ptr == NULL)
	{
		*view_ptr = new ITMView(calib, rgbImage->noDims, rawDepthImage->noDims, true);
		if (this->shortImage != NULL) delete this->shortImage;
		this->shortImage = new ITMShortImage(rawDepthImage->noDims, true, true);
	}

	ITMView *view = *view_ptr;

	view->rgb->SetFrom(rgbImage, MemoryBlock<Vector4u>::CPU_TO_CUDA);
	this->shortImage->SetFrom(rawDepthImage, MemoryBlock<short>::CPU_TO_CUDA);

	switch (inputImageType)
	{
	case InfiniTAM_DISPARITY_IMAGE:
		this->ConvertDisparityToDepth(view->depth, this->shortImage, &(view->calib->intrinsics_d), &(view->calib->disparityCalib));
		break;
	case InfiniTAM_SHORT_DEPTH_IMAGE:
		this->ConvertDepthMMToFloat(view->depth, this->shortImage);
		break;
	default:
		break;
	}
}

void ITMViewBuilder_CUDA::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage)
{
	if (*view_ptr == NULL)
		*view_ptr = new ITMView(calib, rgbImage->noDims, depthImage->noDims, true);

	ITMView *view = *view_ptr;

	view->rgb->UpdateDeviceFromHost();
	view->depth->UpdateDeviceFromHost();
}

void ITMViewBuilder_CUDA::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, ITMIMUMeasurement *imuMeasurement)
{
	if (*view_ptr == NULL) 
	{
		*view_ptr = new ITMViewIMU(calib, rgbImage->noDims, depthImage->noDims, true);
		if (this->shortImage != NULL) delete this->shortImage;
		this->shortImage = new ITMShortImage(depthImage->noDims, true, true);
	}

	ITMViewIMU* imuView = (ITMViewIMU*)(*view_ptr);
	imuView->imu->SetFrom(imuMeasurement);

	this->UpdateView(view_ptr, rgbImage, depthImage);
}

void ITMViewBuilder_CUDA::ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const ITMIntrinsics *depthIntrinsics,
	const ITMDisparityCalib *disparityCalib)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CUDA);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CUDA);

	Vector2f disparityCalibParams; float fx_depth;
	disparityCalibParams.x = disparityCalib->params.x;
	disparityCalibParams.y = disparityCalib->params.y;
	fx_depth = depthIntrinsics->projectionParamsSimple.fx;

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)blockSize.x), (int)ceil((float)imgSize.y / (float)blockSize.y));

	convertDisparityToDepth_device << <gridSize, blockSize >> >(d_out, d_in, disparityCalibParams, fx_depth, imgSize);
}

void ITMViewBuilder_CUDA::ConvertDepthMMToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CUDA);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)blockSize.x), (int)ceil((float)imgSize.y / (float)blockSize.y));

	convertDepthMMToFloat_device << <gridSize, blockSize >> >(d_out, d_in, imgSize);
}

// device functions

__global__ void convertDisparityToDepth_device(float *d_out, const short *d_in, Vector2f disparityCalibParams, float fx_depth, Vector2i imgSize)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x;
	int y = threadIdx.y + blockIdx.y * blockDim.y;

	if ((x >= imgSize.x) || (y >= imgSize.y)) return;

	convertDisparityToDepth(d_out, x, y, d_in, disparityCalibParams, fx_depth, imgSize);
}

__global__ void convertDepthMMToFloat_device(float *d_out, const short *d_in, Vector2i imgSize)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x;
	int y = threadIdx.y + blockIdx.y * blockDim.y;

	if ((x >= imgSize.x) || (y >= imgSize.y)) return;

	convertDepthMMToFloat(d_out, x, y, d_in, imgSize);
}
