// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMLowLevelEngine_CUDA.h"

#include "../Shared/ITMLowLevelEngine_Shared.h"
#include "../../../Utils/ITMCUDAUtils.h"
#include "../../../../ORUtils/CUDADefines.h"

using namespace ITMLib;

ITMLowLevelEngine_CUDA::ITMLowLevelEngine_CUDA(void)
{
	ORcudaSafeCall(cudaMalloc((void**)&counterTempData_device, sizeof(int)));
	ORcudaSafeCall(cudaMallocHost((void**)&counterTempData_host, sizeof(int)));
}

ITMLowLevelEngine_CUDA::~ITMLowLevelEngine_CUDA(void)
{
	ORcudaSafeCall(cudaFree(counterTempData_device));
	ORcudaSafeCall(cudaFreeHost(counterTempData_host));
}

__global__ void convertColourToIntensity_device(float *imageData_out, Vector2i dims, const Vector4u *imageData_in);

__global__ void boxFilter2x2_device(float *imageData_out, const float *imageData_in, Vector2i dims);

__global__ void filterSubsample_device(float *imageData_out, Vector2i newDims, const float *imageData_in, Vector2i oldDims);
__global__ void filterSubsample_device(Vector4u *imageData_out, Vector2i newDims, const Vector4u *imageData_in, Vector2i oldDims);

__global__ void filterSubsampleWithHoles_device(float *imageData_out, Vector2i newDims, const float *imageData_in, Vector2i oldDims);
__global__ void filterSubsampleWithHoles_device(Vector4f *imageData_out, Vector2i newDims, const Vector4f *imageData_in, Vector2i oldDims);

__global__ void gradientX_device(Vector4s *grad, const Vector4u *image, Vector2i imgSize);
__global__ void gradientY_device(Vector4s *grad, const Vector4u *image, Vector2i imgSize);
__global__ void gradientXY_device(Vector2f *grad, const float *image, Vector2i imgSize);

__global__ void countValidDepths_device(const float *imageData_in, int imgSizeTotal, int *counterTempData_device);

// host methods

void ITMLowLevelEngine_CUDA::CopyImage(ITMUChar4Image *image_out, const ITMUChar4Image *image_in) const
{
	Vector4u *dest = image_out->GetData(MEMORYDEVICE_CUDA);
	const Vector4u *src = image_in->GetData(MEMORYDEVICE_CUDA);

	ORcudaSafeCall(cudaMemcpy(dest, src, image_in->dataSize * sizeof(Vector4u), cudaMemcpyDeviceToDevice));
}

void ITMLowLevelEngine_CUDA::CopyImage(ITMFloatImage *image_out, const ITMFloatImage *image_in) const
{
	float *dest = image_out->GetData(MEMORYDEVICE_CUDA);
	const float *src = image_in->GetData(MEMORYDEVICE_CUDA);

	ORcudaSafeCall(cudaMemcpy(dest, src, image_in->dataSize * sizeof(float), cudaMemcpyDeviceToDevice));
}

void ITMLowLevelEngine_CUDA::CopyImage(ITMFloat4Image *image_out, const ITMFloat4Image *image_in) const
{
	Vector4f *dest = image_out->GetData(MEMORYDEVICE_CUDA);
	const Vector4f *src = image_in->GetData(MEMORYDEVICE_CUDA);

	ORcudaSafeCall(cudaMemcpy(dest, src, image_in->dataSize * sizeof(Vector4f), cudaMemcpyDeviceToDevice));
}

void ITMLowLevelEngine_CUDA::ConvertColourToIntensity(ITMFloatImage *image_out, const ITMUChar4Image *image_in) const
{
	const Vector2i dims = image_in->noDims;
	image_out->ChangeDims(dims);

	float *dest = image_out->GetData(MEMORYDEVICE_CUDA);
	const Vector4u *src = image_in->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)dims.x / (float)blockSize.x), (int)ceil((float)dims.y / (float)blockSize.y));

	convertColourToIntensity_device << <gridSize, blockSize >> >(dest, dims, src);
	ORcudaKernelCheck;
}

void ITMLowLevelEngine_CUDA::FilterIntensity(ITMFloatImage *image_out, const ITMFloatImage *image_in) const
{
	Vector2i dims = image_in->noDims;

	image_out->ChangeDims(dims);
	image_out->Clear(0);

	const float *imageData_in = image_in->GetData(MEMORYDEVICE_CUDA);
	float *imageData_out = image_out->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)dims.x / (float)blockSize.x), (int)ceil((float)dims.y / (float)blockSize.y));

	boxFilter2x2_device << <gridSize, blockSize >> >(imageData_out, imageData_in, dims);
	ORcudaKernelCheck;
}

void ITMLowLevelEngine_CUDA::FilterSubsample(ITMUChar4Image *image_out, const ITMUChar4Image *image_in) const
{
	Vector2i oldDims = image_in->noDims;
	Vector2i newDims; newDims.x = image_in->noDims.x / 2; newDims.y = image_in->noDims.y / 2;

	image_out->ChangeDims(newDims);

	const Vector4u *imageData_in = image_in->GetData(MEMORYDEVICE_CUDA);
	Vector4u *imageData_out = image_out->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)newDims.x / (float)blockSize.x), (int)ceil((float)newDims.y / (float)blockSize.y));

	filterSubsample_device << <gridSize, blockSize >> >(imageData_out, newDims, imageData_in, oldDims);
	ORcudaKernelCheck;
}

void ITMLowLevelEngine_CUDA::FilterSubsample(ITMFloatImage *image_out, const ITMFloatImage *image_in) const
{
	Vector2i oldDims = image_in->noDims;
	Vector2i newDims; newDims.x = image_in->noDims.x / 2; newDims.y = image_in->noDims.y / 2;

	image_out->ChangeDims(newDims);
	image_out->Clear(0);

	const float *imageData_in = image_in->GetData(MEMORYDEVICE_CUDA);
	float *imageData_out = image_out->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)newDims.x / (float)blockSize.x), (int)ceil((float)newDims.y / (float)blockSize.y));

	filterSubsample_device << <gridSize, blockSize >> >(imageData_out, newDims, imageData_in, oldDims);
	ORcudaKernelCheck;
}

void ITMLowLevelEngine_CUDA::FilterSubsampleWithHoles(ITMFloatImage *image_out, const ITMFloatImage *image_in) const
{
	Vector2i oldDims = image_in->noDims;
	Vector2i newDims; newDims.x = image_in->noDims.x / 2; newDims.y = image_in->noDims.y / 2;

	image_out->ChangeDims(newDims);

	const float *imageData_in = image_in->GetData(MEMORYDEVICE_CUDA);
	float *imageData_out = image_out->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)newDims.x / (float)blockSize.x), (int)ceil((float)newDims.y / (float)blockSize.y));

	filterSubsampleWithHoles_device << <gridSize, blockSize >> >(imageData_out, newDims, imageData_in, oldDims);
	ORcudaKernelCheck;
}

void ITMLowLevelEngine_CUDA::FilterSubsampleWithHoles(ITMFloat4Image *image_out, const ITMFloat4Image *image_in) const
{
	Vector2i oldDims = image_in->noDims;
	Vector2i newDims; newDims.x = image_in->noDims.x / 2; newDims.y = image_in->noDims.y / 2;

	image_out->ChangeDims(newDims);

	const Vector4f *imageData_in = image_in->GetData(MEMORYDEVICE_CUDA);
	Vector4f *imageData_out = image_out->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)newDims.x / (float)blockSize.x), (int)ceil((float)newDims.y / (float)blockSize.y));

	filterSubsampleWithHoles_device << <gridSize, blockSize >> >(imageData_out, newDims, imageData_in, oldDims);
	ORcudaKernelCheck;
}

void ITMLowLevelEngine_CUDA::GradientX(ITMShort4Image *grad_out, const ITMUChar4Image *image_in) const
{
	grad_out->ChangeDims(image_in->noDims);
	Vector2i imgSize = image_in->noDims;

	Vector4s *grad = grad_out->GetData(MEMORYDEVICE_CUDA);
	const Vector4u *image = image_in->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)blockSize.x), (int)ceil((float)imgSize.y / (float)blockSize.y));

	ORcudaSafeCall(cudaMemset(grad, 0, imgSize.x * imgSize.y * sizeof(Vector4s)));

	gradientX_device << <gridSize, blockSize >> >(grad, image, imgSize);
	ORcudaKernelCheck;
}

void ITMLowLevelEngine_CUDA::GradientY(ITMShort4Image *grad_out, const ITMUChar4Image *image_in) const
{
	grad_out->ChangeDims(image_in->noDims);
	Vector2i imgSize = image_in->noDims;

	Vector4s *grad = grad_out->GetData(MEMORYDEVICE_CUDA);
	const Vector4u *image = image_in->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)blockSize.x), (int)ceil((float)imgSize.y / (float)blockSize.y));

	ORcudaSafeCall(cudaMemset(grad, 0, imgSize.x * imgSize.y * sizeof(Vector4s)));

	gradientY_device << <gridSize, blockSize >> >(grad, image, imgSize);
	ORcudaKernelCheck;
}

void ITMLowLevelEngine_CUDA::GradientXY(ITMFloat2Image *grad_out, const ITMFloatImage *image_in) const
{
	Vector2i imgSize = image_in->noDims;
	grad_out->ChangeDims(imgSize);
	grad_out->Clear();

	Vector2f *grad = grad_out->GetData(MEMORYDEVICE_CUDA);
	const float *image = image_in->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)blockSize.x), (int)ceil((float)imgSize.y / (float)blockSize.y));

	gradientXY_device << <gridSize, blockSize >> >(grad, image, imgSize);
	ORcudaKernelCheck;
}

int ITMLowLevelEngine_CUDA::CountValidDepths(const ITMFloatImage *image_in) const
{
	const float *imageData_in = image_in->GetData(MEMORYDEVICE_CUDA);
	Vector2i imgSize = image_in->noDims;

	dim3 blockSize(256);
	dim3 gridSize((int)ceil((float)imgSize.x*imgSize.y / (float)blockSize.x));

	ORcudaSafeCall(cudaMemset(counterTempData_device, 0, sizeof(int)));
	countValidDepths_device <<<gridSize, blockSize>>>(imageData_in, imgSize.x*imgSize.y, counterTempData_device);
	ORcudaKernelCheck;
	ORcudaSafeCall(cudaMemcpy(counterTempData_host, counterTempData_device, sizeof(int), cudaMemcpyDeviceToHost));

	return *counterTempData_host;
}

// device functions

__global__ void convertColourToIntensity_device(float *imageData_out, Vector2i dims, const Vector4u *imageData_in)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x > dims.x - 1 || y > dims.y - 1) return;

	convertColourToIntensity(imageData_out, x, y, dims, imageData_in);
}

__global__ void boxFilter2x2_device(float *imageData_out, const float *imageData_in, Vector2i dims)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x >= dims.x - 2 || y >= dims.y - 2 || x <= 1 || y <= 1) return;

	boxFilter2x2(imageData_out, x, y, dims, imageData_in, x, y, dims);
}

__global__ void filterSubsample_device(Vector4u *imageData_out, Vector2i newDims, const Vector4u *imageData_in, Vector2i oldDims)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x > newDims.x - 1 || y > newDims.y - 1) return;

	filterSubsample(imageData_out, x, y, newDims, imageData_in, oldDims);
}

__global__ void filterSubsample_device(float *imageData_out, Vector2i newDims, const float *imageData_in, Vector2i oldDims)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x > newDims.x - 2 || y > newDims.y - 2 || x < 1 || y < 1) return;

	boxFilter2x2(imageData_out, x, y, newDims, imageData_in, x * 2, y * 2, oldDims);
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

	if (x < 1 || x > imgSize.x - 2 || y < 1 || y > imgSize.y - 2) return;

	gradientX(grad, x, y, image, imgSize);
}

__global__ void gradientY_device(Vector4s *grad, const Vector4u *image, Vector2i imgSize)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x < 1 || x > imgSize.x - 2 || y < 1 || y > imgSize.y - 2) return;

	gradientY(grad, x, y, image, imgSize);
}

__global__ void gradientXY_device(Vector2f *grad, const float *image, Vector2i imgSize)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x < 1 || x > imgSize.x - 2 || y < 1 || y > imgSize.y - 2) return;

	gradientXY(grad, x, y, image, imgSize);
}

__global__ void countValidDepths_device(const float *imageData_in, int imgSizeTotal, int *counterTempData_device)
{
	int i = threadIdx.x + blockIdx.x * blockDim.x;
	int locId_local = threadIdx.x;

	__shared__ int dim_shared[256];
	//__shared__ bool should_prefix;

	//should_prefix = false;
	//__syncthreads();

	bool isValidPoint = false;

	if (i < imgSizeTotal)
	{
		if (imageData_in[i] > 0.0f) isValidPoint = true;
	}

	//__syncthreads();
	//if (!should_prefix) return;

	dim_shared[locId_local] = isValidPoint;
	__syncthreads();

	if (locId_local < 128) dim_shared[locId_local] += dim_shared[locId_local + 128];
	__syncthreads();
	if (locId_local < 64) dim_shared[locId_local] += dim_shared[locId_local + 64];
	__syncthreads();

	if (locId_local < 32) warpReduce(dim_shared, locId_local);

	if (locId_local == 0) atomicAdd(counterTempData_device, dim_shared[locId_local]);
}

