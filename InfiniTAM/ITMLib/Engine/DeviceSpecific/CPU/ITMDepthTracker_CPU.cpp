// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDepthTracker_CPU.h"
#include "../../DeviceAgnostic/ITMDepthTracker.h"

using namespace ITMLib::Engine;

ITMDepthTracker_CPU::ITMDepthTracker_CPU(Vector2i imgSize, int noHierarchyLevels, int noICPRunTillLevel, int noRotationOnlyLevels, float distThresh, ITMLowLevelEngine *lowLevelEngine)
	:ITMDepthTracker(imgSize, noHierarchyLevels, noRotationOnlyLevels, noICPRunTillLevel, distThresh, lowLevelEngine, false) { }

ITMDepthTracker_CPU::~ITMDepthTracker_CPU(void) { }

void ITMDepthTracker_CPU::ChangeIgnorePixelToZero(ITMFloatImage *image)
{
	Vector2i dims = image->noDims;
	float *imageData = image->GetData(false);

	for (int i = 0; i < dims.x * dims.y; i++) if (imageData[i] < 0.0f) imageData[i] = 0.0f;
}

int ITMDepthTracker_CPU::ComputeGandH(ITMSceneHierarchyLevel *sceneHierarchyLevel, ITMTemplatedHierarchyLevel<ITMFloatImage> *viewHierarchyLevel,
	Matrix4f approxInvPose, Matrix4f scenePose, bool rotationOnly)
{
	int noValidPoints;

	Vector4f *pointsMap = sceneHierarchyLevel->pointsMap->GetData(false);
	Vector4f *normalsMap = sceneHierarchyLevel->normalsMap->GetData(false);
	Vector4f sceneIntrinsics = sceneHierarchyLevel->intrinsics;
	Vector2i sceneImageSize = sceneHierarchyLevel->pointsMap->noDims;

	float *depth = viewHierarchyLevel->depth->GetData(false);
	Vector4f viewIntrinsics = viewHierarchyLevel->intrinsics;
	Vector2i viewImageSize = viewHierarchyLevel->depth->noDims;

	float packedATA[6 * 6];
	int noPara = rotationOnly ? 3 : 6, noParaSQ = rotationOnly ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

	noValidPoints = 0; memset(ATA_host, 0, sizeof(float) * 6 * 6); memset(ATb_host, 0, sizeof(float) * 6);
	memset(packedATA, 0, sizeof(float) * noParaSQ);

	for (int y = 0; y < viewImageSize.y; y++) for (int x = 0; x < viewImageSize.x; x++)
	{
		float localHessian[6 + 5 + 4 + 3 + 2 + 1], localNabla[6];

		for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
		for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;

		bool isValidPoint = computePerPointGH_Depth(localNabla, localHessian, x, y, depth, viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics,
			approxInvPose, scenePose, pointsMap, normalsMap, distThresh, rotationOnly, noPara);

		noValidPoints += (int)isValidPoint;
		for (int i = 0; i < noPara; i++) ATb_host[i] += localNabla[i];
		for (int i = 0; i < noParaSQ; i++) packedATA[i] += localHessian[i];
	}

	for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++) ATA_host[r + c * 6] = packedATA[counter];
	for (int r = 0; r < noPara; ++r) for (int c = r + 1; c < noPara; c++) ATA_host[r + c * 6] = ATA_host[c + r * 6];

	return noValidPoints;
}
