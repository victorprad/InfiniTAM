// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDepthTracker_CUDA.h"
#include "ITMCUDADefines.h"
#include "ITMCUDAUtils.h"
#include "../../DeviceAgnostic/ITMDepthTracker.h"

using namespace ITMLib::Engine;

__global__ void changeIgnorePixelToZero_device(float *imageData_out, Vector2i imgSize);

__global__ void depthTrackerOneLevel_g_rt_device(int *noValidPoints, float *ATA, float *ATb, float *depth, Matrix4f approxInvPose, Vector4f *pointsMap,
	Vector4f *normalsMap, Vector4f sceneIntrinsics, Vector2i sceneImageSize, Matrix4f scenePose, Vector4f viewIntrinsics, Vector2i viewImageSize,
	float distThresh, bool rotationOnly);

// host methods

ITMDepthTracker_CUDA::ITMDepthTracker_CUDA(Vector2i imgSize, int noHierarchyLevels, int noRotationOnlyLevels, int noICPRunTillLevel,  float distThresh, ITMLowLevelEngine *lowLevelEngine)
	:ITMDepthTracker(imgSize, noHierarchyLevels, noRotationOnlyLevels, noICPRunTillLevel, distThresh, lowLevelEngine, true)
{
	int dim_g = 6;
	int dim_h = 6 + 5 + 4 + 3 + 2 + 1;

	Vector2i gridSize((imgSize.x+15)/16, (imgSize.y+15)/16);

	na_host = new int[gridSize.x * gridSize.y];
	g_host = new float[dim_g * gridSize.x * gridSize.y];
	h_host = new float[dim_h * gridSize.x * gridSize.y];

	ITMSafeCall(cudaMalloc((void**)&na_device, sizeof(int)* gridSize.x * gridSize.y));
	ITMSafeCall(cudaMalloc((void**)&g_device, sizeof(float)* dim_g * gridSize.x * gridSize.y));
	ITMSafeCall(cudaMalloc((void**)&h_device, sizeof(float)* dim_h * gridSize.x * gridSize.y));
}

ITMDepthTracker_CUDA::~ITMDepthTracker_CUDA(void)
{
	delete[] na_host;
	delete[] g_host;
	delete[] h_host;

	ITMSafeCall(cudaFree(na_device));
	ITMSafeCall(cudaFree(g_device));
	ITMSafeCall(cudaFree(h_device));
}

void ITMDepthTracker_CUDA::ChangeIgnorePixelToZero(ITMFloatImage *image)
{
	Vector2i dims = image->noDims;
	float *imageData = image->GetData(true);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)dims.x / (float)blockSize.x), (int)ceil((float)dims.y / (float)blockSize.y));

	changeIgnorePixelToZero_device << <gridSize, blockSize >> >(imageData, dims);
}

int ITMDepthTracker_CUDA::ComputeGandH(ITMSceneHierarchyLevel *sceneHierarchyLevel, ITMTemplatedHierarchyLevel<ITMFloatImage> *viewHierarchyLevel,
	Matrix4f approxInvPose, Matrix4f scenePose, bool rotationOnly)
{
	int noValidPoints;

	Vector4f *pointsMap = sceneHierarchyLevel->pointsMap->GetData(true);
	Vector4f *normalsMap = sceneHierarchyLevel->normalsMap->GetData(true);
	Vector4f sceneIntrinsics = sceneHierarchyLevel->intrinsics;
	Vector2i sceneImageSize = sceneHierarchyLevel->pointsMap->noDims;

	float *depth = viewHierarchyLevel->depth->GetData(true);
	Vector4f viewIntrinsics = viewHierarchyLevel->intrinsics;
	Vector2i viewImageSize = viewHierarchyLevel->depth->noDims;

	float packedATA[6 * 6];
	int noPara = rotationOnly ? 3 : 6, noParaSQ = rotationOnly ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)viewImageSize.x / (float)blockSize.x), (int)ceil((float)viewImageSize.y / (float)blockSize.y));

	int gridSizeTotal = gridSize.x * gridSize.y;

	ITMSafeCall(cudaMemset(na_device, 0, gridSizeTotal * sizeof(int)));
	ITMSafeCall(cudaMemset(h_device, 0, gridSizeTotal * noParaSQ * sizeof(float)));
	ITMSafeCall(cudaMemset(g_device, 0, gridSizeTotal * noPara * sizeof(float)));

	depthTrackerOneLevel_g_rt_device << <gridSize, blockSize >> >(na_device, h_device, g_device, depth, approxInvPose, pointsMap,
		normalsMap, sceneIntrinsics, sceneImageSize, scenePose, viewIntrinsics, viewImageSize, distThresh, rotationOnly);

	ITMSafeCall(cudaMemcpy(na_host, na_device, sizeof(int)* gridSizeTotal, cudaMemcpyDeviceToHost));
	ITMSafeCall(cudaMemcpy(h_host, h_device, sizeof(float)* gridSizeTotal * noParaSQ, cudaMemcpyDeviceToHost));
	ITMSafeCall(cudaMemcpy(g_host, g_device, sizeof(float)* gridSizeTotal * noPara, cudaMemcpyDeviceToHost));

	noValidPoints = 0; memset(ATA_host, 0, sizeof(float) * 6 * 6); memset(ATb_host, 0, sizeof(float) * 6);
	memset(packedATA, 0, sizeof(float) * noParaSQ);

	for (int i = 0; i < gridSizeTotal; i++)
	{
		noValidPoints += na_host[i];
		for (int p = 0; p < noPara; p++) ATb_host[p] += g_host[i * noPara + p];
		for (int p = 0; p < noParaSQ; p++) packedATA[p] += h_host[i * noParaSQ + p];
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

__global__ void depthTrackerOneLevel_g_rt_device(int *noValidPoints, float *ATA, float *ATb, float *depth, Matrix4f approxInvPose, Vector4f *pointsMap,
	Vector4f *normalsMap, Vector4f sceneIntrinsics, Vector2i sceneImageSize, Matrix4f scenePose, Vector4f viewIntrinsics, Vector2i viewImageSize,
	float distThresh, bool rotationOnly)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	int locId_local = threadIdx.x + threadIdx.y * blockDim.x;
	int blockId_global = blockIdx.x + blockIdx.y * gridDim.x;
	__shared__ float dim_shared[256];

	dim_shared[locId_local] = 0;
	__syncthreads();

	float localNabla[6], localHessian[21]; bool isValidPoint = false;

	int noPara = rotationOnly ? 3 : 6, noParaSQ = rotationOnly ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

	for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
	for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;

	if (x >= 0 && x < viewImageSize.x && y >= 0 && y < viewImageSize.y)
	{
		isValidPoint = computePerPointGH_Depth(localNabla, localHessian, x, y, depth, viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics,
			approxInvPose, scenePose, pointsMap, normalsMap, distThresh, rotationOnly, noPara);
	}

	//reduction for noValidPoints
	{
		dim_shared[locId_local] = isValidPoint;
		__syncthreads();

		if (locId_local < 128) dim_shared[locId_local] += dim_shared[locId_local + 128];
		__syncthreads();
		if (locId_local < 64) dim_shared[locId_local] += dim_shared[locId_local + 64];
		__syncthreads();

		if (locId_local < 32) warpReduce(dim_shared, locId_local);

		if (locId_local == 0) noValidPoints[blockId_global] = dim_shared[locId_local];
	}

	__syncthreads();

	//reduction for nabla
	for (int paraId = 0; paraId < noPara; paraId++)
	{
		dim_shared[locId_local] = localNabla[paraId];
		__syncthreads();

		if (locId_local < 128) dim_shared[locId_local] += dim_shared[locId_local + 128];
		__syncthreads();
		if (locId_local < 64) dim_shared[locId_local] += dim_shared[locId_local + 64];
		__syncthreads();

		if (locId_local < 32) warpReduce(dim_shared, locId_local);

		if (locId_local == 0) ATb[blockId_global * noPara + paraId] = dim_shared[locId_local];
	}

	__syncthreads();

	//reduction for hessian
	for (int paraId = 0; paraId < noParaSQ; paraId++)
	{
		dim_shared[locId_local] = localHessian[paraId];
		__syncthreads();

		if (locId_local < 128) dim_shared[locId_local] += dim_shared[locId_local + 128];
		__syncthreads();
		if (locId_local < 64) dim_shared[locId_local] += dim_shared[locId_local + 64];
		__syncthreads();

		if (locId_local < 32) warpReduce(dim_shared, locId_local);

		if (locId_local == 0) ATA[blockId_global * noParaSQ + paraId] = dim_shared[locId_local];

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
