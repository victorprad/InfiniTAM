// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMExtendedTracker_CUDA.h"
#include "../../Utils/ITMCUDAUtils.h"
#include "../Shared/ITMExtendedTracker_Shared.h"
#include "../../../ORUtils/CUDADefines.h"

using namespace ITMLib;

struct ITMExtendedTracker_CUDA::AccuCell {
	int numPoints;
	float f;
	float g[6];
	float h[6+5+4+3+2+1];
};

struct ITMExtendedTracker_KernelParameters {
	ITMExtendedTracker_CUDA::AccuCell *accu;
	float *depth;
	Matrix4f approxInvPose;
	Vector4f *pointsMap;
	Vector4f *normalsMap;
	Vector4f sceneIntrinsics;
	Vector2i sceneImageSize;
	Matrix4f scenePose;
	Vector4f viewIntrinsics;
	Vector2i viewImageSize;
	float spaceThresh;
	float viewFrustum_min, viewFrustum_max;
	int tukeyCutOff, framesToSkip, framesToWeight;
};

template<bool shortIteration, bool rotationOnly, bool useWeights>
__global__ void exDepthTrackerOneLevel_g_rt_device(ITMExtendedTracker_KernelParameters para);

// host methods

ITMExtendedTracker_CUDA::ITMExtendedTracker_CUDA(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
	float terminationThreshold, float failureDetectorThreshold, float viewFrustum_min, float viewFrustum_max, int tukeyCutOff, int framesToSkip, int framesToWeight,
	const ITMLowLevelEngine *lowLevelEngine)
	: ITMExtendedTracker(imgSize, trackingRegime, noHierarchyLevels, terminationThreshold, failureDetectorThreshold, viewFrustum_min, viewFrustum_max, 
	tukeyCutOff, framesToSkip, framesToWeight, lowLevelEngine, MEMORYDEVICE_CUDA)
{
	ORcudaSafeCall(cudaMallocHost((void**)&accu_host, sizeof(AccuCell)));
	ORcudaSafeCall(cudaMalloc((void**)&accu_device, sizeof(AccuCell)));
}

ITMExtendedTracker_CUDA::~ITMExtendedTracker_CUDA(void)
{
	ORcudaSafeCall(cudaFreeHost(accu_host));
	ORcudaSafeCall(cudaFree(accu_device));
}

int ITMExtendedTracker_CUDA::ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose)
{
	Vector2i sceneImageSize = sceneHierarchyLevel->pointsMap->noDims;
	Vector2i viewImageSize = viewHierarchyLevel->depth->noDims;

	if (iterationType == TRACKER_ITERATION_NONE) return 0;

	bool shortIteration = (iterationType == TRACKER_ITERATION_ROTATION) || (iterationType == TRACKER_ITERATION_TRANSLATION);

	int noPara = shortIteration ? 3 : 6;

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)viewImageSize.x / (float)blockSize.x), (int)ceil((float)viewImageSize.y / (float)blockSize.y));

	ORcudaSafeCall(cudaMemset(accu_device, 0, sizeof(AccuCell)));

	struct ITMExtendedTracker_KernelParameters args;
	args.accu = accu_device;
	args.depth = viewHierarchyLevel->depth->GetData(MEMORYDEVICE_CUDA);
	args.approxInvPose = approxInvPose;
	args.pointsMap = sceneHierarchyLevel->pointsMap->GetData(MEMORYDEVICE_CUDA);
	args.normalsMap = sceneHierarchyLevel->normalsMap->GetData(MEMORYDEVICE_CUDA);
	args.sceneIntrinsics = sceneHierarchyLevel->intrinsics;
	args.sceneImageSize = sceneImageSize;
	args.scenePose = scenePose;
	args.viewIntrinsics = viewHierarchyLevel->intrinsics;
	args.viewImageSize = viewHierarchyLevel->depth->noDims;
	args.spaceThresh = spaceThresh[levelId];
	args.viewFrustum_min = viewFrustum_min;
	args.viewFrustum_max = viewFrustum_max;
	args.tukeyCutOff = tukeyCutOff;
	args.framesToSkip = framesToSkip;
	args.framesToWeight = framesToWeight;

	//printf("%f %f\n", viewFrustum_min, viewFrustum_max);

	if (currentFrameNo < 100)
	{
		switch (iterationType)
		{
		case TRACKER_ITERATION_ROTATION:
			exDepthTrackerOneLevel_g_rt_device<true, true, false> << <gridSize, blockSize >> >(args);
			ORcudaKernelCheck;
			break;
		case TRACKER_ITERATION_TRANSLATION:
			exDepthTrackerOneLevel_g_rt_device<true, false, false> << <gridSize, blockSize >> >(args);
			ORcudaKernelCheck;
			break;
		case TRACKER_ITERATION_BOTH:
			exDepthTrackerOneLevel_g_rt_device<false, false, false> << <gridSize, blockSize >> >(args);
			ORcudaKernelCheck;
			break;
		default: break;
		}
	}
	else
	{
		switch (iterationType)
		{
		case TRACKER_ITERATION_ROTATION:
			exDepthTrackerOneLevel_g_rt_device<true, true, true> << <gridSize, blockSize >> >(args);
			ORcudaKernelCheck;
			break;
		case TRACKER_ITERATION_TRANSLATION:
			exDepthTrackerOneLevel_g_rt_device<true, false, true> << <gridSize, blockSize >> >(args);
			ORcudaKernelCheck;
			break;
		case TRACKER_ITERATION_BOTH:
			exDepthTrackerOneLevel_g_rt_device<false, false, true> << <gridSize, blockSize >> >(args);
			ORcudaKernelCheck;
			break;
		default: break;
		}
	}

	ORcudaSafeCall(cudaMemcpy(accu_host, accu_device, sizeof(AccuCell), cudaMemcpyDeviceToHost));

	for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++) hessian[r + c * 6] = accu_host->h[counter];
	for (int r = 0; r < noPara; ++r) for (int c = r + 1; c < noPara; c++) hessian[r + c * 6] = hessian[c + r * 6];

	memcpy(nabla, accu_host->g, noPara * sizeof(float));
	f = (accu_host->numPoints > 100) ? accu_host->f / accu_host->numPoints : 1e5f;

	return accu_host->numPoints;
}

// device functions

// huber norm

__device__ float rho(float r, float huber_b)
{
	float tmp = fabs(r) - huber_b;
	tmp = MAX(tmp, 0.0f);
	return r*r - tmp*tmp;
}

__device__ float rho_deriv(float r, float huber_b)
{
	return 2.0f * CLAMP(r, -huber_b, huber_b);
}

__device__ float rho_deriv2(float r, float huber_b)
{
	if (fabs(r) < huber_b) return 2.0f;
	return 0.0f;
}

template<bool shortIteration, bool rotationOnly, bool useWeights>
__device__ void exDepthTrackerOneLevel_g_rt_device_main(ITMExtendedTracker_CUDA::AccuCell *accu, float *depth,
	Matrix4f approxInvPose, Vector4f *pointsMap, Vector4f *normalsMap, Vector4f sceneIntrinsics, Vector2i sceneImageSize, Matrix4f scenePose, 
	Vector4f viewIntrinsics, Vector2i viewImageSize, float spaceThresh, float viewFrustum_min, float viewFrustum_max, 
	int tukeyCutOff, int framesToSkip, int framesToWeight)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	int locId_local = threadIdx.x + threadIdx.y * blockDim.x;

	__shared__ float dim_shared1[256];
	__shared__ float dim_shared2[256];
	__shared__ float dim_shared3[256];
	__shared__ bool should_prefix;

	should_prefix = false;
	__syncthreads();

	const int noPara = shortIteration ? 3 : 6;
	const int noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;
	float A[noPara]; float b; float depthWeight = 1.0f;

	bool isValidPoint = false;

	if (x < viewImageSize.x && y < viewImageSize.y)
	{
		isValidPoint = computePerPointGH_exDepth_Ab<shortIteration, rotationOnly, useWeights>(A, b, x, y, depth[x + y * viewImageSize.x], depthWeight,
			viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, spaceThresh, 
			viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight);
		
		if (isValidPoint) should_prefix = true;
	}

	if (!isValidPoint) {
		for (int i = 0; i < noPara; i++) A[i] = 0.0f;
		b = 0.0f;
	}

	__syncthreads();

	if (!should_prefix) return;
	
	{ //reduction for noValidPoints
		dim_shared1[locId_local] = isValidPoint;
		__syncthreads();

		if (locId_local < 128) dim_shared1[locId_local] += dim_shared1[locId_local + 128];
		__syncthreads();
		if (locId_local < 64) dim_shared1[locId_local] += dim_shared1[locId_local + 64];
		__syncthreads();

		if (locId_local < 32) warpReduce(dim_shared1, locId_local);

		if (locId_local == 0) atomicAdd(&(accu->numPoints), (int)dim_shared1[locId_local]);
	}

	{ //reduction for energy function value
		dim_shared1[locId_local] = rho(b, spaceThresh) * depthWeight;
		__syncthreads();

		if (locId_local < 128) dim_shared1[locId_local] += dim_shared1[locId_local + 128];
		__syncthreads();
		if (locId_local < 64) dim_shared1[locId_local] += dim_shared1[locId_local + 64];
		__syncthreads();

		if (locId_local < 32) warpReduce(dim_shared1, locId_local);

		if (locId_local == 0) atomicAdd(&(accu->f), dim_shared1[locId_local]);
	}

	__syncthreads();

	//reduction for nabla
	for (unsigned char paraId = 0; paraId < noPara; paraId+=3)
	{
		dim_shared1[locId_local] = rho_deriv(b, spaceThresh) * depthWeight * A[paraId + 0];
		dim_shared2[locId_local] = rho_deriv(b, spaceThresh) * depthWeight * A[paraId + 1];
		dim_shared3[locId_local] = rho_deriv(b, spaceThresh) * depthWeight * A[paraId + 2];
		__syncthreads();

		if (locId_local < 128) {
			dim_shared1[locId_local] += dim_shared1[locId_local + 128];
			dim_shared2[locId_local] += dim_shared2[locId_local + 128];
			dim_shared3[locId_local] += dim_shared3[locId_local + 128];
		}
		__syncthreads();
		if (locId_local < 64) {
			dim_shared1[locId_local] += dim_shared1[locId_local + 64];
			dim_shared2[locId_local] += dim_shared2[locId_local + 64];
			dim_shared3[locId_local] += dim_shared3[locId_local + 64];
		}
		__syncthreads();

		if (locId_local < 32) {
			warpReduce(dim_shared1, locId_local);
			warpReduce(dim_shared2, locId_local);
			warpReduce(dim_shared3, locId_local);
		}
		__syncthreads();

		if (locId_local == 0) {
			atomicAdd(&(accu->g[paraId+0]), dim_shared1[0]);
			atomicAdd(&(accu->g[paraId+1]), dim_shared2[0]);
			atomicAdd(&(accu->g[paraId+2]), dim_shared3[0]);
		}
	}

	__syncthreads();

	float localHessian[noParaSQ];
#if (defined(__CUDACC__) && defined(__CUDA_ARCH__)) || (defined(__METALC__))
#pragma unroll
#endif
	for (unsigned char r = 0, counter = 0; r < noPara; r++)
	{
#if (defined(__CUDACC__) && defined(__CUDA_ARCH__)) || (defined(__METALC__))
#pragma unroll
#endif
		for (int c = 0; c <= r; c++, counter++) localHessian[counter] = rho_deriv2(b, spaceThresh) * depthWeight * A[r] * A[c];
	}

	//reduction for hessian
	for (unsigned char paraId = 0; paraId < noParaSQ; paraId+=3)
	{
		dim_shared1[locId_local] = localHessian[paraId+0];
		dim_shared2[locId_local] = localHessian[paraId+1];
		dim_shared3[locId_local] = localHessian[paraId+2];
		__syncthreads();

		if (locId_local < 128) {
			dim_shared1[locId_local] += dim_shared1[locId_local + 128];
			dim_shared2[locId_local] += dim_shared2[locId_local + 128];
			dim_shared3[locId_local] += dim_shared3[locId_local + 128];
		}
		__syncthreads();
		if (locId_local < 64) {
			dim_shared1[locId_local] += dim_shared1[locId_local + 64];
			dim_shared2[locId_local] += dim_shared2[locId_local + 64];
			dim_shared3[locId_local] += dim_shared3[locId_local + 64];
		}
		__syncthreads();

		if (locId_local < 32) {
			warpReduce(dim_shared1, locId_local);
			warpReduce(dim_shared2, locId_local);
			warpReduce(dim_shared3, locId_local);
		}
		__syncthreads();

		if (locId_local == 0) {
			atomicAdd(&(accu->h[paraId+0]), dim_shared1[0]);
			atomicAdd(&(accu->h[paraId+1]), dim_shared2[0]);
			atomicAdd(&(accu->h[paraId+2]), dim_shared3[0]);
		}
	}
}

template<bool shortIteration, bool rotationOnly, bool useWeights>
__global__ void exDepthTrackerOneLevel_g_rt_device(ITMExtendedTracker_KernelParameters para)
{
	exDepthTrackerOneLevel_g_rt_device_main<shortIteration, rotationOnly, useWeights>(para.accu, para.depth,
		para.approxInvPose, para.pointsMap, para.normalsMap, para.sceneIntrinsics, para.sceneImageSize, para.scenePose, 
		para.viewIntrinsics, para.viewImageSize, para.spaceThresh, para.viewFrustum_min, para.viewFrustum_max, 
		para.tukeyCutOff, para.framesToSkip, para.framesToWeight);
}

