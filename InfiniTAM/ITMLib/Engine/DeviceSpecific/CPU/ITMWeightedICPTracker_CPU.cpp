// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMWeightedICPTracker_CPU.h"
#include "../../DeviceAgnostic/ITMWeightedICPTracker.h"

using namespace ITMLib::Engine;

ITMWeightedICPTracker_CPU::ITMWeightedICPTracker_CPU(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel,
	float distThresh, float terminationThreshold, const ITMLowLevelEngine *lowLevelEngine) :ITMWeightedICPTracker(imgSize, trackingRegime, noHierarchyLevels,
	noICPRunTillLevel, distThresh, terminationThreshold, lowLevelEngine, MEMORYDEVICE_CPU) { }

ITMWeightedICPTracker_CPU::~ITMWeightedICPTracker_CPU(void) { }

int ITMWeightedICPTracker_CPU::ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose)
{
	Vector4f *pointsMap = sceneHierarchyLevel->pointsMap->GetData(MEMORYDEVICE_CPU);
	Vector4f *normalsMap = sceneHierarchyLevel->normalsMap->GetData(MEMORYDEVICE_CPU);
	Vector4f sceneIntrinsics = sceneHierarchyLevel->intrinsics;
	Vector2i sceneImageSize = sceneHierarchyLevel->pointsMap->noDims;

	float *depth = viewHierarchyLevel->depth->GetData(MEMORYDEVICE_CPU);
	float *weight = weightHierarchyLevel->depth->GetData(MEMORYDEVICE_CPU);
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


	for (int y = 0; y < viewImageSize.y; y++) for (int x = 0; x < viewImageSize.x; x++)
	{
		float localWeight = weight[x + y*viewImageSize.x] > 0 ? minSigmaZ / weight[x + y*viewImageSize.x] * 0.5f + 0.5f : 0.0f;

		float localHessian[6 + 5 + 4 + 3 + 2 + 1], localNabla[6], localF = 0;

		for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
		for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;

		bool isValidPoint;
        
		switch (iterationType)
		{
		case TRACKER_ITERATION_ROTATION:
			isValidPoint = computePerPointGH_wICP<true, true>(localNabla, localHessian, localF, localWeight, x, y, depth[x + y * viewImageSize.x], viewImageSize,
				viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, distThresh[levelId]);
			break;
		case TRACKER_ITERATION_TRANSLATION:
			isValidPoint = computePerPointGH_wICP<true, false>(localNabla, localHessian, localF, localWeight, x, y, depth[x + y * viewImageSize.x], viewImageSize,
				viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, distThresh[levelId]);
			break;
		case TRACKER_ITERATION_BOTH:
			isValidPoint = computePerPointGH_wICP<false, false>(localNabla, localHessian, localF, localWeight, x, y, depth[x + y * viewImageSize.x], viewImageSize,
				viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, distThresh[levelId]);
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
	f = (noValidPoints > 100) ? sqrt(sumF) / noValidPoints : 1e5f;

	return noValidPoints;
}
