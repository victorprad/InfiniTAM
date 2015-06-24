// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMWeightedICPTracker_CUDA.h"
#include "ITMCUDAUtils.h"
#include "../../DeviceAgnostic/ITMWeightedICPTracker.h"
#include "../../../../ORUtils/CUDADefines.h"

using namespace ITMLib::Engine;

struct ITMWeightedICPTracker_CUDA::AccuCell {
	int numPoints;
	float f;
	float g[6];
	float h[6+5+4+3+2+1];
};

template<bool shortIteration, bool rotationOnly>
__global__ void wICPTrackerOneLevel_g_rt_device(ITMWeightedICPTracker_CUDA::AccuCell *accu, float *depth, Matrix4f approxInvPose, Vector4f *pointsMap,
	Vector4f *normalsMap, float* weightMap,float minsigmaZ, Vector4f sceneIntrinsics, Vector2i sceneImageSize, Matrix4f scenePose, Vector4f viewIntrinsics, Vector2i viewImageSize,
	float distThresh);

// host methods

ITMWeightedICPTracker_CUDA::ITMWeightedICPTracker_CUDA(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel,
	float distThresh, float terminationThreshold, const ITMLowLevelEngine *lowLevelEngine)
	:ITMWeightedICPTracker(imgSize, trackingRegime, noHierarchyLevels, noICPRunTillLevel, distThresh, terminationThreshold, lowLevelEngine, MEMORYDEVICE_CUDA)
{
	Vector2i gridSize((imgSize.x+15)/16, (imgSize.y+15)/16);

	accu_host = new AccuCell[gridSize.x * gridSize.y];
	ITMSafeCall(cudaMalloc((void**)&accu_device, sizeof(AccuCell)* gridSize.x * gridSize.y));
}

ITMWeightedICPTracker_CUDA::~ITMWeightedICPTracker_CUDA(void)
{
	delete[] accu_host;
	ITMSafeCall(cudaFree(accu_device));
}

int ITMWeightedICPTracker_CUDA::ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose)
{
	Vector4f *pointsMap = sceneHierarchyLevel->pointsMap->GetData(MEMORYDEVICE_CUDA);
	Vector4f *normalsMap = sceneHierarchyLevel->normalsMap->GetData(MEMORYDEVICE_CUDA);
	Vector4f sceneIntrinsics = sceneHierarchyLevel->intrinsics;
	Vector2i sceneImageSize = sceneHierarchyLevel->pointsMap->noDims;
	
	float *depth = viewHierarchyLevel->depth->GetData(MEMORYDEVICE_CUDA);
	float *weight = weightHierarchyLevel->depth->GetData(MEMORYDEVICE_CUDA);
	//float mindepth = findMinDepth(viewHierarchyLevel->depth);
	float minSigmaZ = 0.0012f;// + 0.0019f*(mindepth - 0.4f)*(mindepth - 0.4f); 

	Vector4f viewIntrinsics = viewHierarchyLevel->intrinsics;
	Vector2i viewImageSize = viewHierarchyLevel->depth->noDims;

	if (iterationType == TRACKER_ITERATION_NONE) return 0;

	bool shortIteration = (iterationType == TRACKER_ITERATION_ROTATION) || (iterationType == TRACKER_ITERATION_TRANSLATION);

	float sumHessian[6 * 6], sumNabla[6], sumF; int noValidPoints;
	int noPara = shortIteration ? 3 : 6, noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

	noValidPoints = 0; sumF = 0.0f;
	memset(sumHessian, 0, sizeof(float) * noParaSQ);
	memset(sumNabla, 0, sizeof(float) * noPara);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)viewImageSize.x / (float)blockSize.x), (int)ceil((float)viewImageSize.y / (float)blockSize.y));

	int gridSizeTotal = gridSize.x * gridSize.y;

	switch (iterationType)
	{
	case TRACKER_ITERATION_ROTATION:
		wICPTrackerOneLevel_g_rt_device<true, true> << <gridSize, blockSize >> >(accu_device, depth, approxInvPose, pointsMap,normalsMap, weight, minSigmaZ,sceneIntrinsics, sceneImageSize, scenePose, viewIntrinsics, viewImageSize, distThresh[levelId]);
		break;
	case TRACKER_ITERATION_TRANSLATION:
		wICPTrackerOneLevel_g_rt_device<true, false> << <gridSize, blockSize >> >(accu_device, depth, approxInvPose, pointsMap, normalsMap, weight, minSigmaZ, sceneIntrinsics, sceneImageSize, scenePose, viewIntrinsics, viewImageSize, distThresh[levelId]);
		break;
	case TRACKER_ITERATION_BOTH:
		wICPTrackerOneLevel_g_rt_device<false, false> << <gridSize, blockSize >> >(accu_device, depth, approxInvPose, pointsMap, normalsMap, weight, minSigmaZ, sceneIntrinsics, sceneImageSize, scenePose, viewIntrinsics, viewImageSize, distThresh[levelId]);
		break;
	default: break;
	}

	ITMSafeCall(cudaMemcpy(accu_host, accu_device, sizeof(AccuCell)* gridSizeTotal, cudaMemcpyDeviceToHost));

	memset(sumHessian, 0, sizeof(float) * noParaSQ);

	for (int i = 0; i < gridSizeTotal; i++)
	{
		noValidPoints += accu_host[i].numPoints;
		sumF += accu_host[i].f;
		for (int p = 0; p < noPara; p++) sumNabla[p] += accu_host[i].g[p];
		for (int p = 0; p < noParaSQ; p++) sumHessian[p] += accu_host[i].h[p];
	}

	for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++) hessian[r + c * 6] = sumHessian[counter];
	for (int r = 0; r < noPara; ++r) for (int c = r + 1; c < noPara; c++) hessian[r + c * 6] = hessian[c + r * 6];

	memcpy(nabla, sumNabla, noPara * sizeof(float));
	f = (noValidPoints > 100) ? sqrt(sumF) / noValidPoints : 1e5f;

	return noValidPoints;
}

// device functions

template<bool shortIteration, bool rotationOnly>
__global__ void wICPTrackerOneLevel_g_rt_device(ITMWeightedICPTracker_CUDA::AccuCell *accu, float *depth, Matrix4f approxInvPose, Vector4f *pointsMap,
	Vector4f *normalsMap, float* weightMap, float minsigmaZ, Vector4f sceneIntrinsics, Vector2i sceneImageSize, Matrix4f scenePose, Vector4f viewIntrinsics, Vector2i viewImageSize,
	float distThresh)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	int locId_local = threadIdx.x + threadIdx.y * blockDim.x;
	int blockId_global = blockIdx.x + blockIdx.y * gridDim.x;
	__shared__ float dim_shared1[256];
	__shared__ float dim_shared2[256];
	__shared__ float dim_shared3[256];
	__shared__ bool should_prefix;

	should_prefix = false;
	__syncthreads();

	const int noPara = shortIteration ? 3 : 6;
	const int noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;
	float localNabla[noPara], localHessian[noParaSQ], localF = 0; bool isValidPoint = false;


	for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
	for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;

	if (x < viewImageSize.x && y < viewImageSize.y)
	{
		float localWeight = weightMap[x + y*viewImageSize.x] > 0 ? 0.5f * minsigmaZ / weightMap[x + y*viewImageSize.x] + 0.5f : 0.0f;
		
		isValidPoint = computePerPointGH_wICP<shortIteration, rotationOnly>(localNabla, localHessian, localF, localWeight, x, y, depth[x + y * viewImageSize.x],
			viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, distThresh);

		if (isValidPoint) should_prefix = true;
	}

	__syncthreads();

	if (should_prefix) {
	
	{ //reduction for noValidPoints
		dim_shared1[locId_local] = isValidPoint;
		__syncthreads();

		if (locId_local < 128) dim_shared1[locId_local] += dim_shared1[locId_local + 128];
		__syncthreads();
		if (locId_local < 64) dim_shared1[locId_local] += dim_shared1[locId_local + 64];
		__syncthreads();

		if (locId_local < 32) warpReduce(dim_shared1, locId_local);

		if (locId_local == 0) accu[blockId_global].numPoints = dim_shared1[locId_local];
	}

	{ //reduction for energy function value
		dim_shared1[locId_local] = localF;
		__syncthreads();

		if (locId_local < 128) dim_shared1[locId_local] += dim_shared1[locId_local + 128];
		__syncthreads();
		if (locId_local < 64) dim_shared1[locId_local] += dim_shared1[locId_local + 64];
		__syncthreads();

		if (locId_local < 32) warpReduce(dim_shared1, locId_local);

		if (locId_local == 0) accu[blockId_global].f = dim_shared1[locId_local];
	}

	__syncthreads();

	//reduction for nabla
	for (unsigned char paraId = 0; paraId < noPara; paraId+=3)
	{
		dim_shared1[locId_local] = localNabla[paraId+0];
		dim_shared2[locId_local]= localNabla[paraId+1];
		dim_shared3[locId_local]= localNabla[paraId+2];
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
			accu[blockId_global].g[paraId+0] = dim_shared1[0];
			accu[blockId_global].g[paraId+1] = dim_shared2[0];
			accu[blockId_global].g[paraId+2] = dim_shared3[0];
		}
	}

	__syncthreads();

	//reduction for hessian
	for (unsigned char paraId = 0; paraId < noParaSQ; paraId+=3)
	{
		dim_shared1[locId_local] = localHessian[paraId+0];
		dim_shared2[locId_local]= localHessian[paraId+1];
		dim_shared3[locId_local]= localHessian[paraId+2];
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
			accu[blockId_global].h[paraId+0] = dim_shared1[0];
			accu[blockId_global].h[paraId+1] = dim_shared2[0];
			accu[blockId_global].h[paraId+2] = dim_shared3[0];
		}

		//int sdataTargetOffset;
		//for (uint s = (blockDim.x * blockDim.y) >> 1; s > 0; s >>= 1)
		//{
		//	if (locId_local < s)
		//	{
		//		sdataTargetOffset = locId_local + s;
		//		dim_shared1[locId_local] += dim_shared1[sdataTargetOffset];
		//	}
		//	__syncthreads();
		//}

		//atomicAdd(&ATA[paraId], dim_shared1[locId_local]);
	}

	}
	else if (locId_local)
	{
		accu[blockId_global].numPoints = 0;
		accu[blockId_global].f = 0;
		for (int i = 0; i < noPara; ++i) accu[blockId_global].g[i] = 0.0f;
		for (int i = 0; i < noParaSQ; ++i) accu[blockId_global].h[i] = 0.0f;
	}
}

