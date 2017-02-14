// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMColorTracker_CUDA.h"
#include "../../Utils/ITMCUDAUtils.h"
#include "../Shared/ITMColorTracker_Shared.h"
#include "../../Utils/ITMPixelUtils.h"

using namespace ITMLib;

__global__ void colorTrackerOneLevel_f_device(Vector2f *out, Vector4f *locations, Vector4f *colours, Vector4u *rgb, int noTotalPoints,
	Matrix4f M, Vector4f projParams, Vector2i imgSize);

__global__ void colorTrackerOneLevel_g_rt_device(float *g_out, float *h_out, Vector4f *locations, Vector4f *colours, Vector4s *gx, Vector4s *gy, Vector4u *rgb,
	int noTotalPoints, Matrix4f M, Vector4f projParams, Vector2i imgSize);
__global__ void colorTrackerOneLevel_g_ro_device(float *g_out, float *h_out, Vector4f *locations, Vector4f *colours, Vector4s *gx, Vector4s *gy, Vector4u *rgb,
	int noTotalPoints, Matrix4f M, Vector4f projParams, Vector2i imgSize);

// host methods

ITMColorTracker_CUDA::ITMColorTracker_CUDA(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, const ITMLowLevelEngine *lowLevelEngine)
	:ITMColorTracker(imgSize, trackingRegime, noHierarchyLevels, lowLevelEngine, MEMORYDEVICE_CUDA)
{ 
	int dim_g = 6;
	int dim_h = 6 + 5 + 4 + 3 + 2 + 1;

	ORcudaSafeCall(cudaMalloc((void**)&f_device, sizeof(Vector2f)* (imgSize.x * imgSize.y / 128)));
	ORcudaSafeCall(cudaMalloc((void**)&g_device, sizeof(float)* dim_g * (imgSize.x * imgSize.y / 128)));
	ORcudaSafeCall(cudaMalloc((void**)&h_device, sizeof(float)* dim_h * (imgSize.x * imgSize.y / 128)));

	f_host = new Vector2f[imgSize.x * imgSize.y / 128];
	g_host = new float[dim_g * imgSize.x * imgSize.y / 128];
	h_host = new float[dim_h * imgSize.x * imgSize.y / 128];
}

ITMColorTracker_CUDA::~ITMColorTracker_CUDA(void) 
{
	ORcudaSafeCall(cudaFree(f_device));
	ORcudaSafeCall(cudaFree(g_device));
	ORcudaSafeCall(cudaFree(h_device));

	delete[] f_host;
	delete[] g_host;
	delete[] h_host;
}

int ITMColorTracker_CUDA::F_oneLevel(float *f, ORUtils::SE3Pose *pose)
{
	int noTotalPoints = trackingState->pointCloud->noTotalPoints;

	Vector4f projParams = view->calib.intrinsics_rgb.projectionParamsSimple.all;
	projParams.x /= 1 << levelId; projParams.y /= 1 << levelId;
	projParams.z /= 1 << levelId; projParams.w /= 1 << levelId;

	Matrix4f M = pose->GetM();

	Vector2i imgSize = viewHierarchy->GetLevel(levelId)->rgb->noDims;

	float scaleForOcclusions, final_f;

	Vector4f *locations = trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CUDA);
	Vector4f *colours = trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CUDA);
	Vector4u *rgb = viewHierarchy->GetLevel(levelId)->rgb->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(128, 1);
	dim3 gridSize((int)ceil((float)noTotalPoints / (float)blockSize.x), 1);

	if (gridSize.x > 0)
	{
		ORcudaSafeCall(cudaMemset(f_device, 0, sizeof(Vector2f) * gridSize.x));

		colorTrackerOneLevel_f_device << <gridSize, blockSize >> >(f_device, locations, colours, rgb, noTotalPoints, M, projParams, imgSize);
		ORcudaKernelCheck;

		ORcudaSafeCall(cudaMemcpy(f_host, f_device, sizeof(Vector2f)* gridSize.x, cudaMemcpyDeviceToHost));
	}

	final_f = 0; countedPoints_valid = 0;
	for (size_t i = 0; i < gridSize.x; i++) { final_f += f_host[i].x; countedPoints_valid += (int)f_host[i].y; }

	if (countedPoints_valid == 0) { final_f = 1e10; scaleForOcclusions = 1.0; }
	else { scaleForOcclusions = (float)noTotalPoints / countedPoints_valid; }

	f[0] = final_f * scaleForOcclusions;

	return countedPoints_valid;
}

void ITMColorTracker_CUDA::G_oneLevel(float *gradient, float *hessian, ORUtils::SE3Pose *pose) const
{
	int noTotalPoints = trackingState->pointCloud->noTotalPoints;

	Vector4f projParams = view->calib.intrinsics_rgb.projectionParamsSimple.all;
	projParams.x /= 1 << levelId; projParams.y /= 1 << levelId;
	projParams.z /= 1 << levelId; projParams.w /= 1 << levelId;

	Matrix4f M = pose->GetM();

	Vector2i imgSize = viewHierarchy->GetLevel(levelId)->rgb->noDims;

	float scaleForOcclusions;

	bool rotationOnly = iterationType == TRACKER_ITERATION_ROTATION;
	int numPara = rotationOnly ? 3 : 6, numParaSQ = rotationOnly ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

	float globalGradient[6], globalHessian[21];
	for (int i = 0; i < numPara; i++) globalGradient[i] = 0.0f;
	for (int i = 0; i < numParaSQ; i++) globalHessian[i] = 0.0f;

	Vector4f *locations = trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CUDA);
	Vector4f *colours = trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CUDA);
	Vector4u *rgb = viewHierarchy->GetLevel(levelId)->rgb->GetData(MEMORYDEVICE_CUDA);
	Vector4s *gx = viewHierarchy->GetLevel(levelId)->gradientX_rgb->GetData(MEMORYDEVICE_CUDA);
	Vector4s *gy = viewHierarchy->GetLevel(levelId)->gradientY_rgb->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(128, 1);
	dim3 gridSize((int)ceil((float)noTotalPoints / (float)blockSize.x), 1);

	if (gridSize.x > 0)
	{
		if (rotationOnly)
		{
			ORcudaSafeCall(cudaMemset(g_device, 0, sizeof(float) * gridSize.x * 3));
			ORcudaSafeCall(cudaMemset(h_device, 0, sizeof(float) * gridSize.x * 6));

			colorTrackerOneLevel_g_ro_device << <gridSize, blockSize >> >(g_device, h_device, locations, colours, gx, gy, rgb, noTotalPoints,
				M, projParams, imgSize);
			ORcudaKernelCheck;
		}
		else
		{
			ORcudaSafeCall(cudaMemset(g_device, 0, sizeof(float) * gridSize.x * 6));
			ORcudaSafeCall(cudaMemset(h_device, 0, sizeof(float) * gridSize.x * 21));

			colorTrackerOneLevel_g_rt_device << <gridSize, blockSize >> >(g_device, h_device, locations, colours, gx, gy, rgb, noTotalPoints,
				M, projParams, imgSize);
			ORcudaKernelCheck;
		}

		ORcudaSafeCall(cudaMemcpy(g_host, g_device, sizeof(float)* gridSize.x * numPara, cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaMemcpy(h_host, h_device, sizeof(float)* gridSize.x * numParaSQ, cudaMemcpyDeviceToHost));
	}

	for (size_t i = 0; i < gridSize.x; i++)
	{
		for (int p = 0; p < numPara; p++) globalGradient[p] += g_host[i * numPara + p];
		for (int p = 0; p < numParaSQ; p++) globalHessian[p] += h_host[i * numParaSQ + p];
	}

	scaleForOcclusions = (float)noTotalPoints / countedPoints_valid;
	if (countedPoints_valid == 0) { scaleForOcclusions = 1.0f; }

	for (int para = 0, counter = 0; para < numPara; para++)
	{
		gradient[para] = globalGradient[para] * scaleForOcclusions;
		for (int col = 0; col <= para; col++, counter++) hessian[para + col * numPara] = globalHessian[counter] * scaleForOcclusions;
	}
	for (int row = 0; row < numPara; row++)
	{
		for (int col = row + 1; col < numPara; col++) hessian[row + col * numPara] = hessian[col + row * numPara];
	}
}

// device functions

__global__ void colorTrackerOneLevel_f_device(Vector2f *out, Vector4f *locations, Vector4f *colours, Vector4u *rgb, int noTotalPoints,
	Matrix4f M, Vector4f projParams, Vector2i imgSize)
{
	int locId_global = threadIdx.x + blockIdx.x * blockDim.x, locId_local = threadIdx.x;

	__shared__ ORUtils::Vector2_<float> out_shared[128];

	out_shared[locId_local].x = 0; out_shared[locId_local].y = 0;
	__syncthreads();

	if (locId_global < noTotalPoints)
	{
		float colorDiffSq = getColorDifferenceSq(locations, colours, rgb, imgSize, locId_global, projParams, M);
		if (colorDiffSq >= 0)
		{
			out_shared[locId_local].x = colorDiffSq;
			out_shared[locId_local].y = 1.0f;
		}
	}

	__syncthreads();

	int sdataTargetOffset;
	for (uint s = blockDim.x >> 1; s > 0; s >>= 1)
	{
		if (threadIdx.x < s)
		{
			sdataTargetOffset = threadIdx.x + s;
			out_shared[locId_local].x += out_shared[sdataTargetOffset].x;
			out_shared[locId_local].y += out_shared[sdataTargetOffset].y;
		}
		__syncthreads();
	}

	if (threadIdx.x == 0) out[blockIdx.x] = out_shared[0];
}

__global__ void colorTrackerOneLevel_g_rt_device(float *g_out, float *h_out, Vector4f *locations, Vector4f *colours, Vector4s *gx, Vector4s *gy, Vector4u *rgb,
	int noTotalPoints, Matrix4f M, Vector4f projParams, Vector2i imgSize)
{
	int locId_global = threadIdx.x + blockIdx.x * blockDim.x, locId_local = threadIdx.x;

	__shared__ float dim_shared[128];

	dim_shared[locId_local] = 0.0f;
	__syncthreads();

	float localGradient[6], localHessian[21];

	int numPara = 6, numParaSQ = 21;

	for (int i = 0; i < numPara; i++) localGradient[i] = 0.0f;
	for (int i = 0; i < numParaSQ; i++) localHessian[i] = 0.0f;

	if (locId_global < noTotalPoints)
	{
		computePerPointGH_rt_Color(localGradient, localHessian, locations, colours, rgb, imgSize, locId_global, 
			projParams, M, gx, gy, numPara, 0);
	}

	for (int paraId = 0; paraId < numPara; paraId++)
	{
		dim_shared[locId_local] = localGradient[paraId];
		__syncthreads();

		int sdataTargetOffset;
		for (uint s = blockDim.x >> 1; s > 32; s >>= 1)
		{
			if (threadIdx.x < s)
			{
				sdataTargetOffset = threadIdx.x + s;
				dim_shared[locId_local] += dim_shared[sdataTargetOffset];
			}
			__syncthreads();
		}

		if (locId_local < 32) warpReduce(dim_shared, locId_local);

		if (threadIdx.x == 0) g_out[blockIdx.x * numPara + paraId] = dim_shared[locId_local];
	}

	__syncthreads();

	for (int paraId = 0; paraId < numParaSQ; paraId++)
	{
		dim_shared[locId_local] = localHessian[paraId];
		__syncthreads();

		int sdataTargetOffset;
		for (uint s = blockDim.x >> 1; s > 32; s >>= 1)
		{
			if (threadIdx.x < s)
			{
				sdataTargetOffset = threadIdx.x + s;
				dim_shared[locId_local] += dim_shared[sdataTargetOffset];
			}
			__syncthreads();
		}
		if (locId_local < 32) warpReduce(dim_shared, locId_local);

		if (threadIdx.x == 0) h_out[blockIdx.x * numParaSQ + paraId] = dim_shared[locId_local];
	}
}

__global__ void colorTrackerOneLevel_g_ro_device(float *g_out, float *h_out, Vector4f *locations, Vector4f *colours, Vector4s *gx, Vector4s *gy, Vector4u *rgb,
	int noTotalPoints, Matrix4f M, Vector4f projParams, Vector2i imgSize)
{
	int locId_global = threadIdx.x + blockIdx.x * blockDim.x, locId_local = threadIdx.x;

	__shared__ float dim_shared[128];

	dim_shared[locId_local] = 0.0f;
	__syncthreads();

	int numPara = 3, numParaSQ = 6;

	float localGradient[3], localHessian[6];

	for (int i = 0; i < numPara; i++) localGradient[i] = 0.0f;
	for (int i = 0; i < numParaSQ; i++) localHessian[i] = 0.0f;

	if (locId_global < noTotalPoints)
	{
		computePerPointGH_rt_Color(localGradient, localHessian, locations, colours, rgb, imgSize, locId_global,
			projParams, M, gx, gy, numPara, 3);
	}

	for (int paraId = 0; paraId < numPara; paraId++)
	{
		dim_shared[locId_local] = localGradient[paraId];
		__syncthreads();

		int sdataTargetOffset;
		for (uint s = blockDim.x >> 1; s > 32; s >>= 1)
		{
			if (threadIdx.x < s)
			{
				sdataTargetOffset = threadIdx.x + s;
				dim_shared[locId_local] += dim_shared[sdataTargetOffset];
			}
			__syncthreads();
		}

		if (locId_local < 32) warpReduce(dim_shared, locId_local);

		if (threadIdx.x == 0) g_out[blockIdx.x * numPara + paraId] = dim_shared[locId_local];
	}

	__syncthreads();

	for (int paraId = 0; paraId < numParaSQ; paraId++)
	{
		dim_shared[locId_local] = localHessian[paraId];
		__syncthreads();

		int sdataTargetOffset;
		for (uint s = blockDim.x >> 1; s > 32; s >>= 1)
		{
			if (threadIdx.x < s)
			{
				sdataTargetOffset = threadIdx.x + s;
				dim_shared[locId_local] += dim_shared[sdataTargetOffset];
			}
			__syncthreads();
		}

		if (locId_local < 32) warpReduce(dim_shared, locId_local);

		if (threadIdx.x == 0) h_out[blockIdx.x * numParaSQ + paraId] = dim_shared[locId_local];
	}
}
