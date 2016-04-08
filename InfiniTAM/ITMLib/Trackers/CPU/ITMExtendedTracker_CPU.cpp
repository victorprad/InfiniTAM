// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMExtendedTracker_CPU.h"
#include "../Shared/ITMExtendedTracker_Shared.h"

using namespace ITMLib;

ITMExtendedTracker_CPU::ITMExtendedTracker_CPU(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
	float terminationThreshold, float failureDetectorThreshold, const ITMLowLevelEngine *lowLevelEngine)
 : ITMExtendedTracker(imgSize, trackingRegime, noHierarchyLevels, terminationThreshold,  failureDetectorThreshold, lowLevelEngine, MEMORYDEVICE_CPU)
{ }

ITMExtendedTracker_CPU::~ITMExtendedTracker_CPU(void) { }

int ITMExtendedTracker_CPU::ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose)
{
	Vector4f *pointsMap = sceneHierarchyLevel->pointsMap->GetData(MEMORYDEVICE_CPU);
	Vector4f *normalsMap = sceneHierarchyLevel->normalsMap->GetData(MEMORYDEVICE_CPU);
	Vector4f sceneIntrinsics = sceneHierarchyLevel->intrinsics;
	Vector2i sceneImageSize = sceneHierarchyLevel->pointsMap->noDims;

	float *depth = viewHierarchyLevel->depth->GetData(MEMORYDEVICE_CPU);
	Vector4f *depthNormals = viewHierarchyLevel->depthNormals->GetData(MEMORYDEVICE_CPU);
	float *depthUncertainty = viewHierarchyLevel->depthUncertainty->GetData(MEMORYDEVICE_CPU);
	Vector4f viewIntrinsics = viewHierarchyLevel->intrinsics;
	Vector2i viewImageSize = viewHierarchyLevel->depth->noDims;

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
				viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, spaceThresh[levelId]);
			break;
		case TRACKER_ITERATION_TRANSLATION:
			isValidPoint = computePerPointGH_exDepth<true, false, false>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight,
				viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, spaceThresh[levelId]);
			break;
		case TRACKER_ITERATION_BOTH:
			isValidPoint = computePerPointGH_exDepth<false, false, false>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight,
				viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, spaceThresh[levelId]);
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
