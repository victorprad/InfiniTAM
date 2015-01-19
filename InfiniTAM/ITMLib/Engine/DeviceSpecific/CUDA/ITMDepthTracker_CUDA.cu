// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDepthTracker_CUDA.h"
#include "ITMCUDAUtils.h"
#include "../../DeviceAgnostic/ITMDepthTracker.h"
#include "../../../../ORUtils/CUDADefines.h"

using namespace ITMLib::Engine;

struct ITMDepthTracker_CUDA::AccuCell {
	int numPoints;
	float g[6];
	float h[6+5+4+3+2+1];
};

__global__ void changeIgnorePixelToZero_device(float *imageData_out, Vector2i imgSize);

template<bool rotationOnly>
__global__ void depthTrackerOneLevel_g_rt_device(ITMDepthTracker_CUDA::AccuCell *accu, float *depth, Matrix4f approxInvPose, Vector4f *pointsMap,
	Vector4f *normalsMap, Vector4f sceneIntrinsics, Vector2i sceneImageSize, Matrix4f scenePose, Vector4f viewIntrinsics, Vector2i viewImageSize,
	float distThresh);

// host methods

ITMDepthTracker_CUDA::ITMDepthTracker_CUDA(Vector2i imgSize, int noHierarchyLevels, int noRotationOnlyLevels, int noICPRunTillLevel,  
	float distThresh, ITMLowLevelEngine *lowLevelEngine)
	:ITMDepthTracker(imgSize, noHierarchyLevels, noRotationOnlyLevels, noICPRunTillLevel, distThresh, lowLevelEngine, MEMORYDEVICE_CUDA)
{
	Vector2i gridSize((imgSize.x+15)/16, (imgSize.y+15)/16);

	accu_host = new AccuCell[gridSize.x * gridSize.y];
	ITMSafeCall(cudaMalloc((void**)&accu_device, sizeof(AccuCell)* gridSize.x * gridSize.y));
}

ITMDepthTracker_CUDA::~ITMDepthTracker_CUDA(void)
{
	delete[] accu_host;
	ITMSafeCall(cudaFree(accu_device));
}

void ITMDepthTracker_CUDA::ChangeIgnorePixelToZero(ITMFloatImage *image)
{
	Vector2i dims = image->noDims;
	float *imageData = image->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)dims.x / (float)blockSize.x), (int)ceil((float)dims.y / (float)blockSize.y));

	changeIgnorePixelToZero_device << <gridSize, blockSize >> >(imageData, dims);
}

int ITMDepthTracker_CUDA::ComputeGandH(ITMSceneHierarchyLevel *sceneHierarchyLevel, ITMTemplatedHierarchyLevel<ITMFloatImage> *viewHierarchyLevel,
	Matrix4f approxInvPose, Matrix4f scenePose, bool rotationOnly)
{
	int noValidPoints;

	Vector4f *pointsMap = sceneHierarchyLevel->pointsMap->GetData(MEMORYDEVICE_CUDA);
	Vector4f *normalsMap = sceneHierarchyLevel->normalsMap->GetData(MEMORYDEVICE_CUDA);
	Vector4f sceneIntrinsics = sceneHierarchyLevel->intrinsics;
	Vector2i sceneImageSize = sceneHierarchyLevel->pointsMap->noDims;

	float *depth = viewHierarchyLevel->depth->GetData(MEMORYDEVICE_CUDA);
	Vector4f viewIntrinsics = viewHierarchyLevel->intrinsics;
	Vector2i viewImageSize = viewHierarchyLevel->depth->noDims;

	float packedATA[6 * 6];
	int noPara = rotationOnly ? 3 : 6, noParaSQ = rotationOnly ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)viewImageSize.x / (float)blockSize.x), (int)ceil((float)viewImageSize.y / (float)blockSize.y));

	int gridSizeTotal = gridSize.x * gridSize.y;

	if (rotationOnly) {
		depthTrackerOneLevel_g_rt_device<true> << <gridSize, blockSize >> >(accu_device, depth, approxInvPose, pointsMap,
			normalsMap, sceneIntrinsics, sceneImageSize, scenePose, viewIntrinsics, viewImageSize, distThresh);
	} else {
		depthTrackerOneLevel_g_rt_device<false> << <gridSize, blockSize >> >(accu_device, depth, approxInvPose, pointsMap,
			normalsMap, sceneIntrinsics, sceneImageSize, scenePose, viewIntrinsics, viewImageSize, distThresh);
	}

	ITMSafeCall(cudaMemcpy(accu_host, accu_device, sizeof(AccuCell)* gridSizeTotal, cudaMemcpyDeviceToHost));

	noValidPoints = 0; memset(ATA_host, 0, sizeof(float) * 6 * 6); memset(ATb_host, 0, sizeof(float) * 6);
	memset(packedATA, 0, sizeof(float) * noParaSQ);

	for (int i = 0; i < gridSizeTotal; i++)
	{
		noValidPoints += accu_host[i].numPoints;
		for (int p = 0; p < noPara; p++) ATb_host[p] += accu_host[i].g[p];
		for (int p = 0; p < noParaSQ; p++) packedATA[p] += accu_host[i].h[p];
	}

	for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++) ATA_host[r + c * 6] = packedATA[counter];
	for (int r = 0; r < noPara; ++r) for (int c = r + 1; c < noPara; c++) ATA_host[r + c * 6] = ATA_host[c + r * 6];

	return noValidPoints;
}

// device functions

__global__ void changeIgnorePixelToZero_device(float *imageData, Vector2i imgSize)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;
	if (x > imgSize.x - 1 || y > imgSize.y - 1) return;
	if (imageData[x + y * imgSize.x] < 0.0f) imageData[x + y * imgSize.x] = 0.0f;
}

template<bool rotationOnly>
__global__ void depthTrackerOneLevel_g_rt_device(ITMDepthTracker_CUDA::AccuCell *accu, float *depth, Matrix4f approxInvPose, Vector4f *pointsMap,
	Vector4f *normalsMap, Vector4f sceneIntrinsics, Vector2i sceneImageSize, Matrix4f scenePose, Vector4f viewIntrinsics, Vector2i viewImageSize,
	float distThresh)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	int locId_local = threadIdx.x + threadIdx.y * blockDim.x;
	int blockId_global = blockIdx.x + blockIdx.y * gridDim.x;
	__shared__ float dim_shared[256];
	__shared__ float dim_shared2[256];
	__shared__ float dim_shared3[256];
	__shared__ bool should_prefix;
	should_prefix = false;
	__syncthreads();

	const int noPara = rotationOnly ? 3 : 6;
	const int noParaSQ = rotationOnly ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;
	float localNabla[noPara], localHessian[noParaSQ]; bool isValidPoint = false;

	for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
	for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;

	if (x < viewImageSize.x && y < viewImageSize.y)
	{
		isValidPoint = computePerPointGH_Depth<rotationOnly>(localNabla, localHessian, x, y, depth, viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics,
			approxInvPose, scenePose, pointsMap, normalsMap, distThresh);
		if (isValidPoint) should_prefix = true;
	}

	__syncthreads();
	if (should_prefix) {
	//reduction for noValidPoints
	{
		dim_shared[locId_local] = isValidPoint;
		__syncthreads();

		if (locId_local < 128) dim_shared[locId_local] += dim_shared[locId_local + 128];
		__syncthreads();
		if (locId_local < 64) dim_shared[locId_local] += dim_shared[locId_local + 64];
		__syncthreads();

		if (locId_local < 32) warpReduce(dim_shared, locId_local);

		if (locId_local == 0) accu[blockId_global].numPoints = dim_shared[locId_local];
	}

	__syncthreads();

	//reduction for nabla
	for (unsigned char paraId = 0; paraId < noPara; paraId+=3)
	{
		dim_shared[locId_local] = localNabla[paraId+0];
		dim_shared2[locId_local]= localNabla[paraId+1];
		dim_shared3[locId_local]= localNabla[paraId+2];
		__syncthreads();

		if (locId_local < 128) {
			dim_shared[locId_local] += dim_shared[locId_local + 128];
			dim_shared2[locId_local] += dim_shared2[locId_local + 128];
			dim_shared3[locId_local] += dim_shared3[locId_local + 128];
		}
		__syncthreads();
		if (locId_local < 64) {
			dim_shared[locId_local] += dim_shared[locId_local + 64];
			dim_shared2[locId_local] += dim_shared2[locId_local + 64];
			dim_shared3[locId_local] += dim_shared3[locId_local + 64];
		}
		__syncthreads();

		if (locId_local < 32) {
			warpReduce(dim_shared, locId_local);
			warpReduce(dim_shared2, locId_local);
			warpReduce(dim_shared3, locId_local);
		}
		__syncthreads();

		if (locId_local == 0) {
			accu[blockId_global].g[paraId+0] = dim_shared[0];
			accu[blockId_global].g[paraId+1] = dim_shared2[0];
			accu[blockId_global].g[paraId+2] = dim_shared3[0];
		}
	}

	__syncthreads();

	//reduction for hessian
	for (unsigned char paraId = 0; paraId < noParaSQ; paraId+=3)
	{
		dim_shared[locId_local] = localHessian[paraId+0];
		dim_shared2[locId_local]= localHessian[paraId+1];
		dim_shared3[locId_local]= localHessian[paraId+2];
		__syncthreads();

		if (locId_local < 128) {
			dim_shared[locId_local] += dim_shared[locId_local + 128];
			dim_shared2[locId_local] += dim_shared2[locId_local + 128];
			dim_shared3[locId_local] += dim_shared3[locId_local + 128];
		}
		__syncthreads();
		if (locId_local < 64) {
			dim_shared[locId_local] += dim_shared[locId_local + 64];
			dim_shared2[locId_local] += dim_shared2[locId_local + 64];
			dim_shared3[locId_local] += dim_shared3[locId_local + 64];
		}
		__syncthreads();

		if (locId_local < 32) {
			warpReduce(dim_shared, locId_local);
			warpReduce(dim_shared2, locId_local);
			warpReduce(dim_shared3, locId_local);
		}
		__syncthreads();

		if (locId_local == 0) {
			accu[blockId_global].h[paraId+0] = dim_shared[0];
			accu[blockId_global].h[paraId+1] = dim_shared2[0];
			accu[blockId_global].h[paraId+2] = dim_shared3[0];
		}

		//int sdataTargetOffset;
		//for (uint s = (blockDim.x * blockDim.y) >> 1; s > 0; s >>= 1)
		//{
		//	if (locId_local < s)
		//	{
		//		sdataTargetOffset = locId_local + s;
		//		dim_shared[locId_local] += dim_shared[sdataTargetOffset];
		//	}
		//	__syncthreads();
		//}

		//atomicAdd(&ATA[paraId], dim_shared[locId_local]);
	}

	}
	else if (locId_local)
	{
		accu[blockId_global].numPoints = 0;
		for (int i = 0; i < noPara; ++i) accu[blockId_global].g[i] = 0.0f;
		for (int i = 0; i < noParaSQ; ++i) accu[blockId_global].h[i] = 0.0f;
	}
}

