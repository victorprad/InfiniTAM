// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMColorTracker_CPU.h"
#include "../Shared/ITMColorTracker_Shared.h"

using namespace ITMLib;

ITMColorTracker_CPU::ITMColorTracker_CPU(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, const ITMLowLevelEngine *lowLevelEngine)
	: ITMColorTracker(imgSize, trackingRegime, noHierarchyLevels, lowLevelEngine, MEMORYDEVICE_CPU) {  }

ITMColorTracker_CPU::~ITMColorTracker_CPU(void) { }

int ITMColorTracker_CPU::F_oneLevel(float *f, ORUtils::SE3Pose *pose)
{
	int noTotalPoints = trackingState->pointCloud->noTotalPoints;

	Vector4f projParams = view->calib.intrinsics_rgb.projectionParamsSimple.all;
	projParams.x /= 1 << levelId; projParams.y /= 1 << levelId;
	projParams.z /= 1 << levelId; projParams.w /= 1 << levelId;

	Matrix4f M = pose->GetM();

	Vector2i imgSize = viewHierarchy->GetLevel(levelId)->rgb->noDims;

	float scaleForOcclusions, final_f;

	Vector4f *locations = trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CPU);
	Vector4f *colours = trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CPU);
	Vector4u *rgb = viewHierarchy->GetLevel(levelId)->rgb->GetData(MEMORYDEVICE_CPU);

	final_f = 0; countedPoints_valid = 0;
	for (int locId = 0; locId < noTotalPoints; locId++)
	{
		float colorDiffSq = getColorDifferenceSq(locations, colours, rgb, imgSize, locId, projParams, M);
		if (colorDiffSq >= 0) { final_f += colorDiffSq; countedPoints_valid++; }
	}

	if (countedPoints_valid == 0) { final_f = 1e10; scaleForOcclusions = 1.0; }
	else { scaleForOcclusions = (float)noTotalPoints / countedPoints_valid; }

	f[0] = final_f * scaleForOcclusions;

	return countedPoints_valid;
}

void ITMColorTracker_CPU::G_oneLevel(float *gradient, float *hessian, ORUtils::SE3Pose *pose) const
{
	int noTotalPoints = trackingState->pointCloud->noTotalPoints;

	Vector4f projParams = view->calib.intrinsics_rgb.projectionParamsSimple.all;
	projParams.x /= 1 << levelId; projParams.y /= 1 << levelId;
	projParams.z /= 1 << levelId; projParams.w /= 1 << levelId;

	Matrix4f M = pose->GetM();

	Vector2i imgSize = viewHierarchy->GetLevel(levelId)->rgb->noDims;

	float scaleForOcclusions;

	bool rotationOnly = iterationType == TRACKER_ITERATION_ROTATION;
	int numPara = rotationOnly ? 3 : 6, startPara = rotationOnly ? 3 : 0, numParaSQ = rotationOnly ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

	float globalGradient[6], globalHessian[21];
	for (int i = 0; i < numPara; i++) globalGradient[i] = 0.0f;
	for (int i = 0; i < numParaSQ; i++) globalHessian[i] = 0.0f;

	Vector4f *locations = trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CPU);
	Vector4f *colours = trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CPU);
	Vector4u *rgb = viewHierarchy->GetLevel(levelId)->rgb->GetData(MEMORYDEVICE_CPU);
	Vector4s *gx = viewHierarchy->GetLevel(levelId)->gradientX_rgb->GetData(MEMORYDEVICE_CPU);
	Vector4s *gy = viewHierarchy->GetLevel(levelId)->gradientY_rgb->GetData(MEMORYDEVICE_CPU);

	for (int locId = 0; locId < noTotalPoints; locId++)
	{
		float localGradient[6], localHessian[21];

		computePerPointGH_rt_Color(localGradient, localHessian, locations, colours, rgb, imgSize, locId,
			projParams, M, gx, gy, 6, 0);

		bool isValidPoint = computePerPointGH_rt_Color(localGradient, localHessian, locations, colours, rgb, imgSize, locId,
			projParams, M, gx, gy, numPara, startPara);

		if (isValidPoint)
		{
			for (int i = 0; i < numPara; i++) globalGradient[i] += localGradient[i];
			for (int i = 0; i < numParaSQ; i++) globalHessian[i] += localHessian[i];
		}
	}

	scaleForOcclusions = (float)noTotalPoints / countedPoints_valid;
	if (countedPoints_valid == 0) { scaleForOcclusions = 1.0f; }

	for (int para = 0, counter = 0; para < numPara; para++)
	{
		gradient[para] = globalGradient[para] * scaleForOcclusions;
		for (int col = 0; col <= para; col++, counter++) hessian[para + col * numPara] = globalHessian[counter] * scaleForOcclusions;
	}
	for (int row = 0; row < numPara; row++)
	{
		for (int col = row + 1; col < numPara; col++) hessian[row + col * numPara] = hessian[col + row * numPara];
	}
}
