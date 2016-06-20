// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMExtendedTracker_CPU.h"
#include "../Shared/ITMExtendedTracker_Shared.h"

using namespace ITMLib;

ITMExtendedTracker_CPU::ITMExtendedTracker_CPU(Vector2i imgSize_d, Vector2i imgSize_rgb, bool useDepth, bool useColour,
	TrackerIterationType *trackingRegime, int noHierarchyLevels,
	float terminationThreshold, float failureDetectorThreshold, float viewFrustum_min, float viewFrustum_max, int tukeyCutOff, int framesToSkip, int framesToWeight,
	const ITMLowLevelEngine *lowLevelEngine)
	: ITMExtendedTracker(imgSize_d, imgSize_rgb, useDepth, useColour, trackingRegime, noHierarchyLevels, terminationThreshold, failureDetectorThreshold, viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip,
	framesToWeight, lowLevelEngine, MEMORYDEVICE_CPU)
{ }

ITMExtendedTracker_CPU::~ITMExtendedTracker_CPU(void) { }

int ITMExtendedTracker_CPU::ComputeGandH_Depth(float &f, float *nabla, float *hessian, Matrix4f approxInvPose)
{
	Vector4f *pointsMap = sceneHierarchyLevel_Depth->pointsMap->GetData(MEMORYDEVICE_CPU);
	Vector4f *normalsMap = sceneHierarchyLevel_Depth->normalsMap->GetData(MEMORYDEVICE_CPU);
	Vector4f sceneIntrinsics = sceneHierarchyLevel_Depth->intrinsics;
	Vector2i sceneImageSize = sceneHierarchyLevel_Depth->pointsMap->noDims;

	float *depth = viewHierarchyLevel_Depth->depth->GetData(MEMORYDEVICE_CPU);
	Vector4f viewIntrinsics = viewHierarchyLevel_Depth->intrinsics;
	Vector2i viewImageSize = viewHierarchyLevel_Depth->depth->noDims;

	if (iterationType == TRACKER_ITERATION_NONE) return 0;

	bool shortIteration = (iterationType == TRACKER_ITERATION_ROTATION) || (iterationType == TRACKER_ITERATION_TRANSLATION);

	float sumHessian[6 * 6], sumNabla[6], sumF; int noValidPoints;
	int noPara = shortIteration ? 3 : 6, noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

	noValidPoints = 0; sumF = 0.0f;
	memset(sumHessian, 0, sizeof(float) * noParaSQ);
	memset(sumNabla, 0, sizeof(float) * noPara);

	for (int y = 0; y < viewImageSize.y; y++) for (int x = 0; x < viewImageSize.x; x++)
	{
		float localHessian[6 + 5 + 4 + 3 + 2 + 1], localNabla[6], localF = 0;

		for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
		for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;

		bool isValidPoint;

		float depthWeight;

		switch (iterationType)
		{
		case TRACKER_ITERATION_ROTATION:
			isValidPoint = computePerPointGH_exDepth<true, true, false>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight,
				viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, spaceThresh[levelId],
				viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight);
			break;
		case TRACKER_ITERATION_TRANSLATION:
			isValidPoint = computePerPointGH_exDepth<true, false, false>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight,
				viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, spaceThresh[levelId],
				viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight);
			break;
		case TRACKER_ITERATION_BOTH:
			isValidPoint = computePerPointGH_exDepth<false, false, false>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight,
				viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, spaceThresh[levelId],
				viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight);
			break;
		default:
			isValidPoint = false;
			break;
		}

		if (isValidPoint)
		{
			noValidPoints++; sumF += localF;
			for (int i = 0; i < noPara; i++) sumNabla[i] += localNabla[i];
			for (int i = 0; i < noParaSQ; i++) sumHessian[i] += localHessian[i];
		}
	}

	for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++) hessian[r + c * 6] = sumHessian[counter];
	for (int r = 0; r < noPara; ++r) for (int c = r + 1; c < noPara; c++) hessian[r + c * 6] = hessian[c + r * 6];

	memcpy(nabla, sumNabla, noPara * sizeof(float));
	f = (noValidPoints > 100) ? sumF / noValidPoints : 1e5f;

	return noValidPoints;
}

int ITMExtendedTracker_CPU::ComputeGandH_RGB(float &f, float *nabla, float *hessian, Matrix4f approxPose)
{
	Vector2i sceneImageSize = sceneHierarchyLevel_RGB->pointsMap->noDims;
	Vector2i viewImageSize = viewHierarchyLevel_RGB->rgb_current->noDims;

	Vector4f *locations = sceneHierarchyLevel_RGB->pointsMap->GetData(MEMORYDEVICE_CPU);
	Vector4u *rgb_model = viewHierarchyLevel_RGB->rgb_prev->GetData(MEMORYDEVICE_CPU);
	Vector4u *rgb_live = viewHierarchyLevel_RGB->rgb_current->GetData(MEMORYDEVICE_CPU);
	Vector4s *gx = viewHierarchyLevel_RGB->gX->GetData(MEMORYDEVICE_CPU);
	Vector4s *gy = viewHierarchyLevel_RGB->gY->GetData(MEMORYDEVICE_CPU);

	Vector4f projParams = viewHierarchyLevel_RGB->intrinsics;

	if (iterationType == TRACKER_ITERATION_NONE) return 0;

	bool shortIteration = (iterationType == TRACKER_ITERATION_ROTATION) || (iterationType == TRACKER_ITERATION_TRANSLATION);

	float sumHessian[6 * 6], sumNabla[6], sumF; int noValidPoints;
	int noPara = shortIteration ? 3 : 6, noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

	noValidPoints = 0; sumF = 0.0f;
	memset(sumHessian, 0, sizeof(float) * noParaSQ);
	memset(sumNabla, 0, sizeof(float) * noPara);

	for (int y = 0; y < viewImageSize.y; y++) for (int x = 0; x < viewImageSize.x; x++)
	{
		float localHessian[6 + 5 + 4 + 3 + 2 + 1], localNabla[6], localF = 0;

		for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
		for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;

		bool isValidPoint = false;

		if (iterationType != TRACKER_ITERATION_TRANSLATION) // TODO translation not implemented yet
		{
//			isValidPoint = computePerPointGH_exRGB_Ab(localNabla, localF, localHessian,
//				locations[x + y * sceneImageSize.x], rgb_model, rgb_live, viewImageSize, x, y,
//				projParams, approxPose, scenePose, gx, gy, noPara);
		}

		if (isValidPoint)
		{
			noValidPoints++;
			sumF += localF;
			for (int i = 0; i < noPara; i++) sumNabla[i] += localNabla[i];
			for (int i = 0; i < noParaSQ; i++) sumHessian[i] += localHessian[i];
		}
	}

	for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++) hessian[r + c * 6] = sumHessian[counter];
	for (int r = 0; r < noPara; ++r) for (int c = r + 1; c < noPara; c++) hessian[r + c * 6] = hessian[c + r * 6];

	memcpy(nabla, sumNabla, noPara * sizeof(float));
	f = (noValidPoints > 100) ? sumF / noValidPoints : 1e5f;

	return noValidPoints;
}
