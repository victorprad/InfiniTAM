// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDepthTracker_CPU.h"
#include "../../DeviceAgnostic/ITMDepthTracker.h"

using namespace ITMLib::Engine;

ITMDepthTracker_CPU::ITMDepthTracker_CPU(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel,
	float distThresh, const ITMLowLevelEngine *lowLevelEngine) :ITMDepthTracker(imgSize, trackingRegime, noHierarchyLevels, 
	noICPRunTillLevel, distThresh, lowLevelEngine, MEMORYDEVICE_CPU) { }

ITMDepthTracker_CPU::~ITMDepthTracker_CPU(void) { }

int ITMDepthTracker_CPU::ComputeGandH(ITMSceneHierarchyLevel *sceneHierarchyLevel, ITMTemplatedHierarchyLevel<ITMFloatImage> *viewHierarchyLevel,
	Matrix4f approxInvPose, Matrix4f scenePose, TrackerIterationType iterationType)
{
	int noValidPoints;

	Vector4f *pointsMap = sceneHierarchyLevel->pointsMap->GetData(MEMORYDEVICE_CPU);
	Vector4f *normalsMap = sceneHierarchyLevel->normalsMap->GetData(MEMORYDEVICE_CPU);
	Vector4f sceneIntrinsics = sceneHierarchyLevel->intrinsics;
	Vector2i sceneImageSize = sceneHierarchyLevel->pointsMap->noDims;

	float *depth = viewHierarchyLevel->depth->GetData(MEMORYDEVICE_CPU);
	Vector4f viewIntrinsics = viewHierarchyLevel->intrinsics;
	Vector2i viewImageSize = viewHierarchyLevel->depth->noDims;

	noValidPoints = 0; memset(ATA_host, 0, sizeof(float) * 6 * 6); memset(ATb_host, 0, sizeof(float) * 6); f = 0;
	if (iterationType == TRACKER_ITERATION_NONE) return 0;

	bool shortIteration = (iterationType == TRACKER_ITERATION_ROTATION) || (iterationType == TRACKER_ITERATION_TRANSLATION);

	float packedATA[6 * 6];
	int noPara = shortIteration ? 3 : 6, noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

	memset(packedATA, 0, sizeof(float) * noParaSQ);

	for (int y = 0; y < viewImageSize.y; y++) for (int x = 0; x < viewImageSize.x; x++)
	{
		float localHessian[6 + 5 + 4 + 3 + 2 + 1], localNabla[6], localF = 0;

		for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
		for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;

		bool isValidPoint;
        
		switch (iterationType)
		{
		case TRACKER_ITERATION_ROTATION:
			isValidPoint = computePerPointGH_Depth<true, true>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], viewImageSize,
				viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, distThresh);
			break;
		case TRACKER_ITERATION_TRANSLATION:
			isValidPoint = computePerPointGH_Depth<true, false>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], viewImageSize,
				viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, distThresh);
			break;
		case TRACKER_ITERATION_BOTH:
			isValidPoint = computePerPointGH_Depth<false, false>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], viewImageSize,
				viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, distThresh);
			break;
		default: break;
		}

		if (isValidPoint)
		{
			noValidPoints++; f += localF;
			for (int i = 0; i < noPara; i++) ATb_host[i] += localNabla[i];
			for (int i = 0; i < noParaSQ; i++) packedATA[i] += localHessian[i];
		}
	}

	for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++) ATA_host[r + c * 6] = packedATA[counter];
	for (int r = 0; r < noPara; ++r) for (int c = r + 1; c < noPara; c++) ATA_host[r + c * 6] = ATA_host[c + r * 6];

	return noValidPoints;
}
