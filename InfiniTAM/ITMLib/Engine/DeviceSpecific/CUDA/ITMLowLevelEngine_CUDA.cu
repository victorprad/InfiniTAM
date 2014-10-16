// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMLowLevelEngine_CUDA.h"
#include "ITMCUDADefines.h"

#include "../../DeviceAgnostic/ITMLowLevelEngine.h"

using namespace ITMLib::Engine;

ITMLowLevelEngine_CUDA::ITMLowLevelEngine_CUDA(void) { }
ITMLowLevelEngine_CUDA::~ITMLowLevelEngine_CUDA(void) { }

__global__ void convertDisparityToDepth_device(float *depth_out, const short *depth_in, Vector2f disparityCalibParams, float fx_depth, Vector2i imgSize);

__global__ void convertDepthMMToFloat_device(float *d_out, const short *d_in, Vector2i imgSize);

__global__ void filterSubsample_device(Vector4u *imageData_out, Vector2i newDims, const Vector4u *imageData_in, Vector2i oldDims);

__global__ void filterSubsampleWithHoles_device(float *imageData_out, Vector2i newDims, const float *imageData_in, Vector2i oldDims);
__global__ void filterSubsampleWithHoles_device(Vector4f *imageData_out, Vector2i newDims, const Vector4f *imageData_in, Vector2i oldDims);

__global__ void gradientX_device(Vector4s *grad, const Vector4u *image, Vector2i imgSize);
__global__ void gradientY_device(Vector4s *grad, const Vector4u *image, Vector2i imgSize);

// host methods

void ITMLowLevelEngine_CUDA::CopyImage(ITMUChar4Image *image_out, const ITMUChar4Image *image_in)
{
	Vector4u *dest = image_out->GetData(true);
	const Vector4u *src = image_in->GetData(true);

	ITMSafeCall(cudaMemcpy(dest, src, image_in->dataSize * sizeof(Vector4u), cudaMemcpyDeviceToDevice));
}

void ITMLowLevelEngine_CUDA::CopyImage(ITMFloatImage *image_out, const ITMFloatImage *image_in)
{
	float *dest = image_out->GetData(true);
	const float *src = image_in->GetData(true);

	ITMSafeCall(cudaMemcpy(dest, src, image_in->dataSize * sizeof(float), cudaMemcpyDeviceToDevice));
}

void ITMLowLevelEngine_CUDA::CopyImage(ITMFloat4Image *image_out, const ITMFloat4Image *image_in)
{
	Vector4f *dest = image_out->GetData(true);
	const Vector4f *src = image_in->GetData(true);

	ITMSafeCall(cudaMemcpy(dest, src, image_in->dataSize * sizeof(Vector4f), cudaMemcpyDeviceToDevice));
}

void ITMLowLevelEngine_CUDA::FilterSubsample(ITMUChar4Image *image_out, const ITMUChar4Image *image_in)
{
	Vector2i oldDims = image_in->noDims;
	Vector2i newDims; newDims.x = image_in->noDims.x / 2; newDims.y = image_in->noDims.y / 2;

	image_out->ChangeDims(newDims);

	const Vector4u *imageData_in = image_in->GetData(true);
	Vector4u *imageData_out = image_out->GetData(true);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)newDims.x / (float)blockSize.x), (int)ceil((float)newDims.y / (float)blockSize.y));

	filterSubsample_device << <gridSize, blockSize >> >(imageData_out, newDims, imageData_in, oldDims);
}

void ITMLowLevelEngine_CUDA::FilterSubsampleWithHoles(ITMFloatImage *image_out, const ITMFloatImage *image_in)
{
	Vector2i oldDims = image_in->noDims;
	Vector2i newDims; newDims.x = image_in->noDims.x / 2; newDims.y = image_in->noDims.y / 2;

	image_out->ChangeDims(newDims);

	const float *imageData_in = image_in->GetData(true);
	float *imageData_out = image_out->GetData(true);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)newDims.x / (float)blockSize.x), (int)ceil((float)newDims.y / (float)blockSize.y));

	filterSubsampleWithHoles_device << <gridSize, blockSize >> >(imageData_out, newDims, imageData_in, oldDims);
}

void ITMLowLevelEngine_CUDA::FilterSubsampleWithHoles(ITMFloat4Image *image_out, const ITMFloat4Image *image_in)
{
	Vector2i oldDims = image_in->noDims;
	Vector2i newDims; newDims.x = image_in->noDims.x / 2; newDims.y = image_in->noDims.y / 2;

	image_out->ChangeDims(newDims);

	const Vector4f *imageData_in = image_in->GetData(true);
	Vector4f *imageData_out = image_out->GetData(true);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)newDims.x / (float)blockSize.x), (int)ceil((float)newDims.y / (float)blockSize.y));

	filterSubsampleWithHoles_device << <gridSize, blockSize >> >(imageData_out, newDims, imageData_in, oldDims);
}

void ITMLowLevelEngine_CUDA::GradientX(ITMShort4Image *grad_out, const ITMUChar4Image *image_in)
{
	grad_out->ChangeDims(image_in->noDims);
	Vector2i imgSize = image_in->noDims;

	Vector4s *grad = grad_out->GetData(true); 
	const Vector4u *image = image_in->GetData(true);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)blockSize.x), (int)ceil((float)imgSize.y / (float)blockSize.y));

	ITMSafeCall(cudaMemset(grad, 0, imgSize.x * imgSize.y * sizeof(Vector4s)));

	gradientX_device << <gridSize, blockSize >> >(grad, image, imgSize);
}

void ITMLowLevelEngine_CUDA::GradientY(ITMShort4Image *grad_out, const ITMUChar4Image *image_in)
{
	grad_out->ChangeDims(image_in->noDims);
	Vector2i imgSize = image_in->noDims;

	Vector4s *grad = grad_out->GetData(true);
	const Vector4u *image = image_in->GetData(true);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)blockSize.x), (int)ceil((float)imgSize.y / (float)blockSize.y));

	ITMSafeCall(cudaMemset(grad, 0, imgSize.x * imgSize.y * sizeof(Vector4s)));

	gradientY_device << <gridSize, blockSize >> >(grad, image, imgSize);
}

void ITMLowLevelEngine_CUDA::ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const ITMIntrinsics *depthIntrinsics,
	const ITMDisparityCalib *disparityCalib)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(true);
	float *d_out = depth_out->GetData(true);

	Vector2f disparityCalibParams; float fx_depth;
	disparityCalibParams.x = disparityCalib->params.x;
	disparityCalibParams.y = disparityCalib->params.y;
	fx_depth = depthIntrinsics->projectionParamsSimple.fx;

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)blockSize.x), (int)ceil((float)imgSize.y / (float)blockSize.y));

	convertDisparityToDepth_device << <gridSize, blockSize >> >(d_out, d_in, disparityCalibParams, fx_depth, imgSize);
}

void ITMLowLevelEngine_CUDA::ConvertDepthMMToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(true);
	float *d_out = depth_out->GetData(true);

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

__global__ void filterSubsample_device(Vector4u *imageData_out, Vector2i newDims, const Vector4u *imageData_in, Vector2i oldDims)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x > newDims.x - 1 || y > newDims.y - 1) return;

	filterSubsample(imageData_out, x, y, newDims, imageData_in, oldDims);
}

__global__ void filterSubsampleWithHoles_device(float *imageData_out, Vector2i newDims, const float *imageData_in, Vector2i oldDims)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x > newDims.x - 1 || y > newDims.y - 1) return;

	filterSubsampleWithHoles(imageData_out, x, y, newDims, imageData_in, oldDims);
}

__global__ void filterSubsampleWithHoles_device(Vector4f *imageData_out, Vector2i newDims, const Vector4f *imageData_in, Vector2i oldDims)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x > newDims.x - 1 || y > newDims.y - 1) return;

	filterSubsampleWithHoles(imageData_out, x, y, newDims, imageData_in, oldDims);
}

__global__ void gradientX_device(Vector4s *grad, const Vector4u *image, Vector2i imgSize)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x < 2 || x > imgSize.x - 2 || y < 2 || y > imgSize.y - 2) return;

	gradientX(grad, x, y, image, imgSize);
}

__global__ void gradientY_device(Vector4s *grad, const Vector4u *image, Vector2i imgSize)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x < 2 || x > imgSize.x - 2 || y < 2 || y > imgSize.y - 2) return;

	gradientY(grad, x, y, image, imgSize);
}
