// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMExtendedTracker_CUDA.h"
#include "../../Utils/ITMCUDAUtils.h"
#include "../Shared/ITMExtendedTracker_Shared.h"
#include "../../../ORUtils/CUDADefines.h"
#include <iostream>

using namespace ITMLib;

struct ITMExtendedTracker_CUDA::AccuCell {
	int numPoints;
	float f;
	float g[6];
	float h[6+5+4+3+2+1];
};

struct ITMExtendedTracker_KernelParameters_Depth {
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

struct ITMExtendedTracker_KernelParameters_RGB {
	ITMExtendedTracker_CUDA::AccuCell *accu;
	Vector4f *pointsMap;
	Vector4s *gx;
	Vector4s *gy;
	Vector4u *rgb_live;
	Vector4f *rgb_model;
	Vector2i viewImageSize;
	Vector2i sceneImageSize;
	Matrix4f approxInvPose;
	Matrix4f approxPose;
	Matrix4f scenePose;
	Vector4f projParams;
	float colourThresh;
	float viewFrustum_min, viewFrustum_max;
	float tukeyCutOff, framesToSkip, framesToWeight;
};

template<bool shortIteration, bool rotationOnly, bool useWeights>
__global__ void exDepthTrackerOneLevel_g_rt_device(ITMExtendedTracker_KernelParameters_Depth para);

template<bool shortIteration, bool rotationOnly, bool useWeights>
__global__ void exRGBTrackerOneLevel_g_rt_device(ITMExtendedTracker_KernelParameters_RGB para);

__global__ void exRGBTrackerProjectPrevImage_device(Vector4f *out_rgb, const Vector4u *in_rgb, const Vector4f *in_points, Vector2i imageSize, Vector2i sceneSize, Vector4f intrinsics, Matrix4f scenePose);

// host methods

ITMExtendedTracker_CUDA::ITMExtendedTracker_CUDA(Vector2i imgSize_d, Vector2i imgSize_rgb, bool useDepth, bool useColour,
	float colourWeight, TrackerIterationType *trackingRegime, int noHierarchyLevels,
	float terminationThreshold, float failureDetectorThreshold, float viewFrustum_min, float viewFrustum_max, int tukeyCutOff, int framesToSkip, int framesToWeight,
	const ITMLowLevelEngine *lowLevelEngine)
	: ITMExtendedTracker(imgSize_d, imgSize_rgb, useDepth, useColour, colourWeight, trackingRegime, noHierarchyLevels, terminationThreshold, failureDetectorThreshold, viewFrustum_min, viewFrustum_max,
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

int ITMExtendedTracker_CUDA::ComputeGandH_Depth(float &f, float *nabla, float *hessian, Matrix4f approxInvPose)
{
	Vector2i sceneImageSize = sceneHierarchyLevel_Depth->pointsMap->noDims;
	Vector2i viewImageSize = viewHierarchyLevel_Depth->depth->noDims;

	if (iterationType == TRACKER_ITERATION_NONE) return 0;

	bool shortIteration = (iterationType == TRACKER_ITERATION_ROTATION) || (iterationType == TRACKER_ITERATION_TRANSLATION);

	int noPara = shortIteration ? 3 : 6;

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)viewImageSize.x / (float)blockSize.x), (int)ceil((float)viewImageSize.y / (float)blockSize.y));

	ORcudaSafeCall(cudaMemset(accu_device, 0, sizeof(AccuCell)));

	struct ITMExtendedTracker_KernelParameters_Depth args;
	args.accu = accu_device;
	args.depth = viewHierarchyLevel_Depth->depth->GetData(MEMORYDEVICE_CUDA);
	args.approxInvPose = approxInvPose;
	args.pointsMap = sceneHierarchyLevel_Depth->pointsMap->GetData(MEMORYDEVICE_CUDA);
	args.normalsMap = sceneHierarchyLevel_Depth->normalsMap->GetData(MEMORYDEVICE_CUDA);
	args.sceneIntrinsics = sceneHierarchyLevel_Depth->intrinsics;
	args.sceneImageSize = sceneImageSize;
	args.scenePose = scenePose;
	args.viewIntrinsics = viewHierarchyLevel_Depth->intrinsics;
	args.viewImageSize = viewHierarchyLevel_Depth->depth->noDims;
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

	if (accu_host->numPoints > 100)
	{
		for (int i = 0; i < 6 * 6; ++i) hessian[i] = hessian[i] / accu_host->numPoints;
		for (int i = 0; i < 6; ++i) nabla[i] = nabla[i] / accu_host->numPoints;

		f = accu_host->f / accu_host->numPoints;
	}
	else
	{
		f = 1e5f;
	}

	return accu_host->numPoints;
}

int ITMExtendedTracker_CUDA::ComputeGandH_RGB(float &f, float *nabla, float *hessian, Matrix4f approxInvPose)
{
	Vector2i sceneImageSize = sceneHierarchyLevel_RGB->pointsMap->noDims;
	Vector2i viewImageSize = viewHierarchyLevel_RGB->rgb_current->noDims;

	sceneHierarchyLevel_RGB->pointsMap->UpdateHostFromDevice();
	previousProjectedRGBLevel->depth->UpdateHostFromDevice();
	viewHierarchyLevel_RGB->rgb_current->UpdateHostFromDevice();
	viewHierarchyLevel_RGB->gX->UpdateHostFromDevice();
	viewHierarchyLevel_RGB->gY->UpdateHostFromDevice();

	Vector4f *locations = sceneHierarchyLevel_RGB->pointsMap->GetData(MEMORYDEVICE_CPU);
	Vector4f *rgb_model = previousProjectedRGBLevel->depth->GetData(MEMORYDEVICE_CPU);
	Vector4u *rgb_live = viewHierarchyLevel_RGB->rgb_current->GetData(MEMORYDEVICE_CPU);
	Vector4s *gx = viewHierarchyLevel_RGB->gX->GetData(MEMORYDEVICE_CPU);
	Vector4s *gy = viewHierarchyLevel_RGB->gY->GetData(MEMORYDEVICE_CPU);

	Vector4f projParams = viewHierarchyLevel_RGB->intrinsics;

	Matrix4f approxPose;
	approxInvPose.inv(approxPose);
//	approxPose = depthToRGBTransform * approxPose;
//	approxPose = approxPose;

	if (iterationType == TRACKER_ITERATION_NONE) return 0;

	bool shortIteration = (iterationType == TRACKER_ITERATION_ROTATION) || (iterationType == TRACKER_ITERATION_TRANSLATION);

	float sumHessian[6 * 6], sumNabla[6], sumF; int noValidPoints;
	int noPara = shortIteration ? 3 : 6, noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

	noValidPoints = 0; sumF = 0.0f;
	memset(sumHessian, 0, sizeof(float) * noParaSQ);
	memset(sumNabla, 0, sizeof(float) * noPara);

	float minF = 1e10, maxF = 0.f;
	float minNabla[6], maxNabla[6];
	float minHessian[noParaSQ], maxHessian[noParaSQ];

	for(int i = 0; i < noPara; ++i)
	{
		minNabla[i] = 1e10f;
		maxNabla[i] = -1e10f;
	}

	for(int i = 0; i < noParaSQ; ++i)
	{
		minHessian[i] = 1e10f;
		maxHessian[i] = -1e10f;
	}

	for (int y = 0; y < viewImageSize.y; y++) for (int x = 0; x < viewImageSize.x; x++)
	{
		float localHessian[6 + 5 + 4 + 3 + 2 + 1], localNabla[6], localF = 0;

		for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
		for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;

		bool isValidPoint = false;
		float depthWeight = 1.f;

		if (iterationType != TRACKER_ITERATION_TRANSLATION) // TODO translation not implemented yet
		{
			if (currentFrameNo < 100)
				isValidPoint = computePerPointGH_exRGB_Ab<false>(localNabla, localF, localHessian, depthWeight,
					locations[x + y * sceneImageSize.x], rgb_model[x + y * sceneImageSize.x], rgb_live, viewImageSize, x, y,
					projParams, approxPose, approxInvPose, scenePose, gx, gy, colourThresh[levelId], viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight, noPara);
			else
				isValidPoint = computePerPointGH_exRGB_Ab<true>(localNabla, localF, localHessian, depthWeight,
					locations[x + y * sceneImageSize.x], rgb_model[x + y * sceneImageSize.x], rgb_live, viewImageSize, x, y,
					projParams, approxPose, approxInvPose, scenePose, gx, gy, colourThresh[levelId], viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight, noPara);
		}

		if (isValidPoint)
		{
			noValidPoints++;
			sumF += localF;
			for (int i = 0; i < noPara; i++) sumNabla[i] += localNabla[i];
			for (int i = 0; i < noParaSQ; i++) sumHessian[i] += localHessian[i];

			minF = MIN(minF, localF);
			maxF = MAX(maxF, localF);

			for (int i = 0; i < noPara; i++)
			{
				minNabla[i] = MIN(minNabla[i], localNabla[i]);
				maxNabla[i] = MAX(maxNabla[i], localNabla[i]);
			}

			for (int i = 0; i < noParaSQ; i++)
			{
				minHessian[i] = MIN(minHessian[i], localHessian[i]);
				maxHessian[i] = MAX(maxHessian[i], localHessian[i]);
			}
		}
	}

	printf("Min F: %g - Max F: %g\n", minF, maxF);
	printf("Min Nabla: ");
	for (int i = 0; i < noPara; i++)
	{
		printf("%g - ", minNabla[i]);
	}
	printf("\nMax Nabla: ");
	for (int i = 0; i < noPara; i++)
	{
		printf("%g - ", maxNabla[i]);
	}
	printf("\n");
	printf("Min Hessian: ");
	for (int i = 0; i < noParaSQ; i++)
	{
		printf("%g - ", minHessian[i]);
	}
	printf("\nMax Hessian: ");
	for (int i = 0; i < noParaSQ; i++)
	{
		printf("%g - ", maxHessian[i]);
	}
	printf("\n\n");

	for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++) hessian[r + c * 6] = sumHessian[counter];
	for (int r = 0; r < noPara; ++r) for (int c = r + 1; c < noPara; c++) hessian[r + c * 6] = hessian[c + r * 6];

	memcpy(nabla, sumNabla, noPara * sizeof(float));

	if (noValidPoints > 100)
	{
		for (int i = 0; i < 6 * 6; ++i) hessian[i] = hessian[i] / noValidPoints;
		for (int i = 0; i < 6; ++i) nabla[i] = nabla[i] / noValidPoints;

		f = sumF / noValidPoints;
	}
	else
	{
		f = 1e5f;
	}

	return noValidPoints;

//	Vector2i sceneImageSize = sceneHierarchyLevel_RGB->pointsMap->noDims;
//	Vector2i viewImageSize = viewHierarchyLevel_RGB->rgb_current->noDims;
//
//	if (iterationType == TRACKER_ITERATION_NONE) return 0;
//
//	Matrix4f approxPose;
//	approxInvPose.inv(approxPose);
//	approxPose = depthToRGBTransform * approxPose;
//
//	bool shortIteration = (iterationType == TRACKER_ITERATION_ROTATION) || (iterationType == TRACKER_ITERATION_TRANSLATION);
//
//	int noPara = shortIteration ? 3 : 6;
//
//	dim3 blockSize(16, 16);
//	dim3 gridSize((int)ceil((float)sceneImageSize.x / (float)blockSize.x), (int)ceil((float)sceneImageSize.y / (float)blockSize.y));
//
//	ORcudaSafeCall(cudaMemset(accu_device, 0, sizeof(AccuCell)));
//
//	struct ITMExtendedTracker_KernelParameters_RGB args;
//	args.accu = accu_device;
//	args.rgb_live = viewHierarchyLevel_RGB->rgb_current->GetData(MEMORYDEVICE_CUDA);
//	args.rgb_model = previousProjectedRGBLevel->depth->GetData(MEMORYDEVICE_CUDA);
//	args.gx = viewHierarchyLevel_RGB->gX->GetData(MEMORYDEVICE_CUDA);
//	args.gy = viewHierarchyLevel_RGB->gY->GetData(MEMORYDEVICE_CUDA);
//	args.pointsMap = sceneHierarchyLevel_RGB->pointsMap->GetData(MEMORYDEVICE_CUDA);
//	args.viewImageSize = viewImageSize;
//	args.sceneImageSize = sceneImageSize;
//	args.approxInvPose = approxInvPose;
//	args.approxPose = approxPose;
//	args.scenePose = scenePose;
//	args.projParams = viewHierarchyLevel_RGB->intrinsics;
//	args.colourThresh = colourThresh[levelId];
//	args.viewFrustum_min = viewFrustum_min;
//	args.viewFrustum_max = viewFrustum_max;
//	args.tukeyCutOff = tukeyCutOff;
//	args.framesToSkip = framesToSkip;
//	args.framesToWeight = framesToWeight;
//
//	if (currentFrameNo < 100)
//	{
//		switch (iterationType)
//		{
//		case TRACKER_ITERATION_ROTATION:
//			exRGBTrackerOneLevel_g_rt_device<true, true, false> << <gridSize, blockSize >> >(args);
//			ORcudaKernelCheck;
//			break;
//		case TRACKER_ITERATION_TRANSLATION:
//			exRGBTrackerOneLevel_g_rt_device<true, false, false> << <gridSize, blockSize >> >(args);
//			ORcudaKernelCheck;
//			break;
//		case TRACKER_ITERATION_BOTH:
//			exRGBTrackerOneLevel_g_rt_device<false, false, false> << <gridSize, blockSize >> >(args);
//			ORcudaKernelCheck;
//			break;
//		default: break;
//		}
//	}
//	else
//	{
//		switch (iterationType)
//		{
//		case TRACKER_ITERATION_ROTATION:
//			exRGBTrackerOneLevel_g_rt_device<true, true, true> << <gridSize, blockSize >> >(args);
//			ORcudaKernelCheck;
//			break;
//		case TRACKER_ITERATION_TRANSLATION:
//			exRGBTrackerOneLevel_g_rt_device<true, false, true> << <gridSize, blockSize >> >(args);
//			ORcudaKernelCheck;
//			break;
//		case TRACKER_ITERATION_BOTH:
//			exRGBTrackerOneLevel_g_rt_device<false, false, true> << <gridSize, blockSize >> >(args);
//			ORcudaKernelCheck;
//			break;
//		default: break;
//		}
//	}
//
//	ORcudaSafeCall(cudaMemcpy(accu_host, accu_device, sizeof(AccuCell), cudaMemcpyDeviceToHost));
//
//	for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++) hessian[r + c * 6] = accu_host->h[counter];
//	for (int r = 0; r < noPara; ++r) for (int c = r + 1; c < noPara; c++) hessian[r + c * 6] = hessian[c + r * 6];
//
//	memcpy(nabla, accu_host->g, noPara * sizeof(float));
//
//	if (accu_host->numPoints > 100)
//	{
//		for (int i = 0; i < 6 * 6; ++i) hessian[i] = hessian[i] / accu_host->numPoints;
//		for (int i = 0; i < 6; ++i) nabla[i] = nabla[i] / accu_host->numPoints;
//
//		f = accu_host->f / accu_host->numPoints;
//	}
//	else
//	{
//		f = 1e5f;
//	}
//
//	return accu_host->numPoints;
}

void ITMExtendedTracker_CUDA::ProjectPreviousRGBFrame(const Matrix4f &scenePose)
{
//	Vector2i imageSize = viewHierarchyLevel_RGB->rgb_prev->noDims;
//	Vector2i sceneSize = sceneHierarchyLevel_RGB->pointsMap->noDims;
//
//	previousProjectedRGBLevel->depth->ChangeDims(sceneSize);
//
//	sceneHierarchyLevel_RGB->pointsMap->UpdateHostFromDevice();
//	viewHierarchyLevel_RGB->rgb_prev->UpdateHostFromDevice();
//	previousProjectedRGBLevel->depth->UpdateHostFromDevice();
//
//	Vector4f projParams = viewHierarchyLevel_RGB->intrinsics;
//	Vector4f *pointsMap = sceneHierarchyLevel_RGB->pointsMap->GetData(MEMORYDEVICE_CPU);
//	Vector4u *rgbIn = viewHierarchyLevel_RGB->rgb_prev->GetData(MEMORYDEVICE_CPU);
//	Vector4f *rgbOut = previousProjectedRGBLevel->depth->GetData(MEMORYDEVICE_CPU);
//
//	for (int y = 0; y < sceneSize.y; y++) for (int x = 0; x < sceneSize.x; x++)
//	{
//		projectPreviousPoint_exRGB(x, y, rgbOut, rgbIn, pointsMap, imageSize, sceneSize, projParams, scenePose);
//	}
//
//	sceneHierarchyLevel_RGB->pointsMap->UpdateDeviceFromHost();
//	viewHierarchyLevel_RGB->rgb_prev->UpdateDeviceFromHost();
//	previousProjectedRGBLevel->depth->UpdateDeviceFromHost();

	sceneHierarchyLevel_RGB->pointsMap->UpdateDeviceFromHost();
	viewHierarchyLevel_RGB->rgb_prev->UpdateDeviceFromHost();
	previousProjectedRGBLevel->depth->UpdateDeviceFromHost();

	Vector2i imageSize = viewHierarchyLevel_RGB->rgb_prev->noDims;
	Vector2i sceneSize = sceneHierarchyLevel_RGB->pointsMap->noDims; // Also the size of the projected image

	previousProjectedRGBLevel->depth->ChangeDims(sceneSize); // Actual reallocation should happen only once per run.

	Vector4f projParams = viewHierarchyLevel_RGB->intrinsics;
	const Vector4f *pointsMap = sceneHierarchyLevel_RGB->pointsMap->GetData(MEMORYDEVICE_CUDA);
	const Vector4u *rgbIn = viewHierarchyLevel_RGB->rgb_prev->GetData(MEMORYDEVICE_CUDA);
	Vector4f *rgbOut = previousProjectedRGBLevel->depth->GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)sceneSize.x / (float)blockSize.x), (int)ceil((float)sceneSize.y / (float)blockSize.y));

	exRGBTrackerProjectPrevImage_device<<<gridSize, blockSize>>>(rgbOut, rgbIn, pointsMap, imageSize, sceneSize, projParams, scenePose);
	ORcudaKernelCheck;

	sceneHierarchyLevel_RGB->pointsMap->UpdateHostFromDevice();
	viewHierarchyLevel_RGB->rgb_prev->UpdateHostFromDevice();
	previousProjectedRGBLevel->depth->UpdateHostFromDevice();
}

// device functions
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

	__syncthreads();

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
__device__ void exRGBTrackerOneLevel_g_rt_device_main(ITMExtendedTracker_CUDA::AccuCell *accu,
	Vector4f *locations, Vector4f *rgb_model, Vector4s *gx, Vector4s *gy, Vector4u *rgb,
	Matrix4f approxPose, Matrix4f approxInvPose, Matrix4f scenePose, Vector4f projParams,
	Vector2i imgSize, Vector2i sceneSize, float colourThresh, float viewFrustum_min, float viewFrustum_max,
	float tukeyCutoff, float framesToSkip, float framesToWeight)
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
	float localHessian[noParaSQ];
	float A[noPara];
	float b;
	float depthWeight = 1.0f;

	bool isValidPoint = false;

	if (x < sceneSize.x && y < sceneSize.y)
	{
		// FIXME Translation only not implemented yet
		if(!shortIteration || rotationOnly)
		{
			isValidPoint = computePerPointGH_exRGB_Ab<useWeights>(A, b, localHessian, depthWeight, locations[x + y * sceneSize.x],
				rgb_model[x + y * sceneSize.x], rgb, imgSize, x, y,	projParams, approxPose, approxInvPose, scenePose, gx, gy,
				colourThresh, viewFrustum_min, viewFrustum_max, tukeyCutoff, framesToSkip, framesToWeight, noPara);
		}

		if (isValidPoint) should_prefix = true;
	}

	if (!isValidPoint)
	{
		for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;
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

	__syncthreads();

	{ //reduction for energy function value
		dim_shared1[locId_local] = b;
		__syncthreads();

		if (locId_local < 128) dim_shared1[locId_local] += dim_shared1[locId_local + 128];
		__syncthreads();
		if (locId_local < 64) dim_shared1[locId_local] += dim_shared1[locId_local + 64];
		__syncthreads();

		if (locId_local < 32) warpReduce(dim_shared1, locId_local);
		__syncthreads();

		if (locId_local == 0) atomicAdd(&(accu->f), dim_shared1[locId_local]);
	}

	__syncthreads();

	//reduction for nabla
	for (unsigned char paraId = 0; paraId < noPara; paraId += 3)
	{
		dim_shared1[locId_local] = A[paraId + 0];
		dim_shared2[locId_local] = A[paraId + 1];
		dim_shared3[locId_local] = A[paraId + 2];
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
			atomicAdd(&(accu->g[paraId + 0]), dim_shared1[0]);
			atomicAdd(&(accu->g[paraId + 1]), dim_shared2[0]);
			atomicAdd(&(accu->g[paraId + 2]), dim_shared3[0]);
		}
	}

	__syncthreads();

	//reduction for hessian
	for (unsigned char paraId = 0; paraId < noParaSQ; paraId += 3)
	{
		dim_shared1[locId_local] = localHessian[paraId + 0];
		dim_shared2[locId_local] = localHessian[paraId + 1];
		dim_shared3[locId_local] = localHessian[paraId + 2];
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
			atomicAdd(&(accu->h[paraId + 0]), dim_shared1[0]);
			atomicAdd(&(accu->h[paraId + 1]), dim_shared2[0]);
			atomicAdd(&(accu->h[paraId + 2]), dim_shared3[0]);
		}
	}
}

template<bool shortIteration, bool rotationOnly, bool useWeights>
__global__ void exDepthTrackerOneLevel_g_rt_device(ITMExtendedTracker_KernelParameters_Depth para)
{
	exDepthTrackerOneLevel_g_rt_device_main<shortIteration, rotationOnly, useWeights>(para.accu, para.depth,
		para.approxInvPose, para.pointsMap, para.normalsMap, para.sceneIntrinsics, para.sceneImageSize, para.scenePose,
		para.viewIntrinsics, para.viewImageSize, para.spaceThresh, para.viewFrustum_min, para.viewFrustum_max,
		para.tukeyCutOff, para.framesToSkip, para.framesToWeight);
}

template<bool shortIteration, bool rotationOnly, bool useWeights>
__global__ void exRGBTrackerOneLevel_g_rt_device(ITMExtendedTracker_KernelParameters_RGB para)
{
	exRGBTrackerOneLevel_g_rt_device_main<shortIteration, rotationOnly, useWeights>(para.accu, para.pointsMap,
		para.rgb_model, para.gx, para.gy, para.rgb_live, para.approxPose, para.approxInvPose, para.scenePose,
		para.projParams, para.viewImageSize, para.sceneImageSize, para.colourThresh, para.viewFrustum_min, para.viewFrustum_max,
		para.tukeyCutOff, para.framesToSkip, para.framesToWeight);
}

__global__ void exRGBTrackerProjectPrevImage_device(Vector4f *out_rgb, const Vector4u *in_rgb, const Vector4f *in_points, Vector2i imageSize, Vector2i sceneSize, Vector4f intrinsics, Matrix4f scenePose)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x;
	int y = threadIdx.y + blockIdx.y * blockDim.y;

	projectPreviousPoint_exRGB(x, y, out_rgb, in_rgb, in_points, imageSize, sceneSize, intrinsics, scenePose);
}
