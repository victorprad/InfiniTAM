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

void ITMViewBuilder_CUDA::UpdateView(ITMView *view, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
	view->rgb->SetFrom(rgbImage, MemoryBlock<Vector4u>::MemoryCopyDirection::CPU_TO_CUDA);
	this->shortImage->SetFrom(rawDepthImage, MemoryBlock<short>::MemoryCopyDirection::CPU_TO_CUDA);

	if (inputImageType == InfiniTAM_DISPARITY_IMAGE)
		this->ConvertDisparityToDepth(view->depth, this->shortImage, &(view->calib->intrinsics_d), &(view->calib->disparityCalib));
	else if (inputImageType == InfiniTAM_SHORT_DEPTH_IMAGE)
		this->ConvertDepthMMToFloat(view->depth, this->shortImage);
}

void ITMViewBuilder_CUDA::UpdateView(ITMView *view, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage)
{
	view->rgb->UpdateDeviceFromHost();
	view->depth->UpdateDeviceFromHost();
}

void ITMViewBuilder_CUDA::AllocateView(ITMView *view, Vector2i imgSize_rgb, Vector2i imgSize_d)
{
	view->calib = new ITMRGBDCalib(*calib);
	view->rgb = new ITMUChar4Image(imgSize_rgb, true, true);
	view->depth = new ITMFloatImage(imgSize_d, true, true);

	if (this->shortImage != NULL) delete this->shortImage;
	this->shortImage = new ITMShortImage(imgSize_d, true, true);

	view->isAllocated = true;
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