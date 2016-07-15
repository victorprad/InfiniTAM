// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMExtendedTracker_CPU.h"
#include "../Shared/ITMExtendedTracker_Shared.h"

using namespace ITMLib;

ITMExtendedTracker_CPU::ITMExtendedTracker_CPU(Vector2i imgSize_d, Vector2i imgSize_rgb, bool useDepth, bool useColour,
	float colourWeight, TrackerIterationType *trackingRegime, int noHierarchyLevels,
	float terminationThreshold, float failureDetectorThreshold, float viewFrustum_min, float viewFrustum_max, int tukeyCutOff, int framesToSkip, int framesToWeight,
	const ITMLowLevelEngine *lowLevelEngine)
	: ITMExtendedTracker(imgSize_d, imgSize_rgb, useDepth, useColour, colourWeight, trackingRegime, noHierarchyLevels, terminationThreshold, failureDetectorThreshold, viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip,
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
}

int ITMExtendedTracker_CPU::ComputeGandH_RGB(float &f, float *nabla, float *hessian, Matrix4f approxInvPose)
{
	Vector2i sceneImageSize = sceneHierarchyLevel_RGB->pointsMap->noDims;
	Vector2i viewImageSize = viewHierarchyLevel_RGB->rgb_current->noDims;

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
}

void ITMExtendedTracker_CPU::ProjectPreviousRGBFrame(const Matrix4f &scenePose)
{
	Vector2i imageSize = viewHierarchyLevel_RGB->rgb_prev->noDims;
	Vector2i sceneSize = sceneHierarchyLevel_RGB->pointsMap->noDims;

	previousProjectedRGBLevel->depth->ChangeDims(sceneSize);

	Vector4f projParams = viewHierarchyLevel_RGB->intrinsics;
	Vector4f *pointsMap = sceneHierarchyLevel_RGB->pointsMap->GetData(MEMORYDEVICE_CPU);
	Vector4u *rgbIn = viewHierarchyLevel_RGB->rgb_prev->GetData(MEMORYDEVICE_CPU);
	Vector4f *rgbOut = previousProjectedRGBLevel->depth->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < sceneSize.y; y++) for (int x = 0; x < sceneSize.x; x++)
	{
		projectPreviousPoint_exRGB(x, y, rgbOut, rgbIn, pointsMap, imageSize, sceneSize, projParams, scenePose);
	}
}
