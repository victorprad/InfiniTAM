// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#ifndef NOMINMAX 
#define NOMINMAX
#endif 

#include "ITMExtendedTracker.h"
#include "../../../ORUtils/Cholesky.h"

#include "../../../ORUtils/FileUtils.h"

#include <math.h>
#include <limits>

using namespace ITMLib;

const int ITMExtendedTracker::MIN_VALID_POINTS_DEPTH = 100;
const int ITMExtendedTracker::MIN_VALID_POINTS_RGB = 100;

ITMExtendedTracker::ITMExtendedTracker(Vector2i imgSize_d,
									   Vector2i imgSize_rgb,
									   bool useDepth,
									   bool useColour,
									   float colourWeight,
									   TrackerIterationType *trackingRegime,
									   int noHierarchyLevels,
									   float terminationThreshold,
									   float failureDetectorThreshold,
									   float viewFrustum_min,
									   float viewFrustum_max,
									   float minColourGradient,
									   float tukeyCutOff,
									   int framesToSkip,
									   int framesToWeight,
									   const ITMLowLevelEngine *lowLevelEngine,
									   MemoryDeviceType memoryType
									   )
{
	this->useDepth = useDepth;
	this->useColour = useColour;

	if (!useDepth && !useColour)
	{
		throw std::runtime_error("Invalid configuration: at least one of depth and colour trackers must be active.");
	}

	// We always need the current depth image pyramid
	viewHierarchy_Depth = new ITMImageHierarchy<ITMDepthHierarchyLevel>(imgSize_d, trackingRegime,
																		noHierarchyLevels, memoryType, true);

	// We need the colour pyramid only when colour is used for tracking.
	if (useColour)
	{
		// Do NOT skip allocation for level 0 since intensity images are only used in the tracker
		viewHierarchy_Intensity = new ITMImageHierarchy<ITMIntensityHierarchyLevel>(imgSize_rgb, trackingRegime,
																					noHierarchyLevels, memoryType,
																					false);

		reprojectedPointsHierarchy = new ITMImageHierarchy<ITMTemplatedHierarchyLevel<ITMFloat4Image> >(imgSize_d, trackingRegime, noHierarchyLevels, memoryType, false);
		projectedIntensityHierarchy = new ITMImageHierarchy<ITMTemplatedHierarchyLevel<ITMFloatImage> >(imgSize_d, trackingRegime, noHierarchyLevels, memoryType, false);
	}
	else
	{
		viewHierarchy_Intensity = NULL;
		reprojectedPointsHierarchy = NULL;
		projectedIntensityHierarchy = NULL;
	}

	// We need the scene hierarchy (ICP and Normal raycasts) only if depth is used for tracking.
	if (useDepth)
	{
		sceneHierarchy = new ITMImageHierarchy<ITMSceneHierarchyLevel>(imgSize_d, trackingRegime, noHierarchyLevels, memoryType, true);
	}

	this->noIterationsPerLevel = new int[noHierarchyLevels];
	this->spaceThresh = new float[noHierarchyLevels];
	this->colourThresh = new float[noHierarchyLevels];

	SetupLevels(noHierarchyLevels * 2, 2, 0.01f, 0.002f, 0.1f, 0.02f);

	this->lowLevelEngine = lowLevelEngine;

	this->terminationThreshold = terminationThreshold;

	this->colourWeight = colourWeight;
	this->minColourGradient = minColourGradient;

	this->viewFrustum_min = viewFrustum_min;
	this->viewFrustum_max = viewFrustum_max;
	this->tukeyCutOff = tukeyCutOff;
	this->framesToSkip = framesToSkip;
	this->framesToWeight = framesToWeight;

	map = new ORUtils::HomkerMap(2);
	svmClassifier = new ORUtils::SVMClassifier(map->getDescriptorSize(4));

	//all below obtained from dataset in matlab
	float w[20];
	w[0] = -3.15813f; w[1] = -2.38038f; w[2] = 1.93359f; w[3] = 1.56642f; w[4] = 1.76306f;
	w[5] = -0.747641f; w[6] = 4.41852f; w[7] = 1.72048f; w[8] = -0.482545f; w[9] = -5.07793f;
	w[10] = 1.98676f; w[11] = -0.45688f; w[12] = 2.53969f; w[13] = -3.50527f; w[14] = -1.68725f;
	w[15] = 2.31608f; w[16] = 5.14778f; w[17] = 2.31334f; w[18] = -14.128f; w[19] = 6.76423f;

	float b = 9.334260e-01f + failureDetectorThreshold;

	mu = Vector4f(-34.9470512137603f, -33.1379108518478f, 0.195948598235857f, 0.611027292662361f);
	sigma = Vector4f(68.1654461020426f, 60.6607826748643f, 0.00343068557187040f, 0.0402595570918749f);

	svmClassifier->SetVectors(w, b);
}

ITMExtendedTracker::~ITMExtendedTracker(void)
{
	delete viewHierarchy_Depth;
	delete viewHierarchy_Intensity;
	delete sceneHierarchy;
	delete reprojectedPointsHierarchy;
	delete projectedIntensityHierarchy;

	delete[] noIterationsPerLevel;
	delete[] spaceThresh;
	delete[] colourThresh;

	delete map;
	delete svmClassifier;
}

void ITMExtendedTracker::SetupLevels(int numIterCoarse, int numIterFine, float spaceThreshCoarse, float spaceThreshFine, float colourThreshCoarse, float colourThreshFine)
{
	int noHierarchyLevels = viewHierarchy_Depth->GetNoLevels();

	if (numIterCoarse != -1 && numIterFine != -1) {
		float step = (float)(numIterCoarse - numIterFine) / (float)(noHierarchyLevels - 1);
		float val = (float)numIterCoarse;
		for (int levelId = noHierarchyLevels - 1; levelId >= 0; levelId--) {
			this->noIterationsPerLevel[levelId] = (int)round(val);
			val -= step;
		}
	}
	if (spaceThreshCoarse >= 0.0f && spaceThreshFine >= 0.0f) {
		float step = (float)(spaceThreshCoarse - spaceThreshFine) / (float)(noHierarchyLevels - 1);
		float val = spaceThreshCoarse;
		for (int levelId = noHierarchyLevels - 1; levelId >= 0; levelId--) {
			this->spaceThresh[levelId] = val;
			val -= step;
		}
	}
	if (colourThreshCoarse >= 0.0f && colourThreshFine >= 0.0f) {
		float step = (float)(colourThreshCoarse - colourThreshFine) / (float)(noHierarchyLevels - 1);
		float val = colourThreshCoarse;
		for (int levelId = noHierarchyLevels - 1; levelId >= 0; levelId--) {
			this->colourThresh[levelId] = val;
			val -= step;
		}
	}
}

void ITMExtendedTracker::SetEvaluationData(ITMTrackingState *trackingState, const ITMView *view)
{
	this->trackingState = trackingState;
	this->view = view;

	// Note: - viewHierarchy_Depth allows pointers to external data at level 0
	// 		 - viewHierarchy_Intensity does not, since the intensity is computed in this tracker
	// 		 - sceneHierarchy allows pointers to external data at level 0

	// Depth image hierarchy is always used
	viewHierarchy_Depth->GetLevel(0)->intrinsics = view->calib.intrinsics_d.projectionParamsSimple.all;
	viewHierarchy_Depth->GetLevel(0)->depth = view->depth;

	if (useColour)
	{
		viewHierarchy_Intensity->GetLevel(0)->intrinsics = view->calib.intrinsics_rgb.projectionParamsSimple.all;

		// Convert RGB to intensity
		lowLevelEngine->ConvertColourToIntensity(viewHierarchy_Intensity->GetLevel(0)->intensity_current, view->rgb);
		lowLevelEngine->ConvertColourToIntensity(viewHierarchy_Intensity->GetLevel(0)->intensity_prev, view->rgb_prev);

		// Compute first level gradients
		lowLevelEngine->GradientXY(viewHierarchy_Intensity->GetLevel(0)->gradients,
								   viewHierarchy_Intensity->GetLevel(0)->intensity_prev);
	}

	// Pointclouds are needed only when the depth tracker is enabled
	if (useDepth)
	{
		sceneHierarchy->GetLevel(0)->intrinsics = view->calib.intrinsics_d.projectionParamsSimple.all;
		sceneHierarchy->GetLevel(0)->pointsMap = trackingState->pointCloud->locations;
		sceneHierarchy->GetLevel(0)->normalsMap = trackingState->pointCloud->colours;
	}

	scenePose = trackingState->pose_pointCloud->GetM();
	depthToRGBTransform = view->calib.trafo_rgb_to_depth.calib_inv;
	framesProcessed = trackingState->framesProcessed;
}

void ITMExtendedTracker::PrepareForEvaluation()
{
	// Create depth pyramid
	for (int i = 1; i < viewHierarchy_Depth->GetNoLevels(); i++)
	{
		ITMDepthHierarchyLevel *currentLevel = viewHierarchy_Depth->GetLevel(i);
		ITMDepthHierarchyLevel *previousLevel = viewHierarchy_Depth->GetLevel(i - 1);

		lowLevelEngine->FilterSubsampleWithHoles(currentLevel->depth, previousLevel->depth);

		currentLevel->intrinsics = previousLevel->intrinsics * 0.5f;
	}

	// Create current and previous frame pyramids
	if (useColour)
	{
		for (int i = 1; i < viewHierarchy_Intensity->GetNoLevels(); i++)
		{
			ITMIntensityHierarchyLevel *currentLevel = viewHierarchy_Intensity->GetLevel(i);
			ITMIntensityHierarchyLevel *previousLevel = viewHierarchy_Intensity->GetLevel(i - 1);

			lowLevelEngine->FilterSubsample(currentLevel->intensity_current, previousLevel->intensity_current);
			lowLevelEngine->FilterSubsample(currentLevel->intensity_prev, previousLevel->intensity_prev);

			currentLevel->intrinsics = previousLevel->intrinsics * 0.5f;

			// Also compute gradients
			lowLevelEngine->GradientXY(currentLevel->gradients, currentLevel->intensity_prev);
		}

		// Project RGB image according to the depth->rgb transform and cache it to speed up the energy computation
		for (int i = 0; i < viewHierarchy_Intensity->GetNoLevels(); ++i)
		{
			ITMTemplatedHierarchyLevel<ITMFloat4Image> *pointsOut = reprojectedPointsHierarchy->GetLevel(i);
			ITMTemplatedHierarchyLevel<ITMFloatImage> *intensityOut = projectedIntensityHierarchy->GetLevel(i);

			const ITMIntensityHierarchyLevel *intensityIn = viewHierarchy_Intensity->GetLevel(i);
			const ITMDepthHierarchyLevel *depthIn = viewHierarchy_Depth->GetLevel(i);

			Vector4f intrinsics_rgb = intensityIn->intrinsics;
			Vector4f intrinsics_depth = depthIn->intrinsics;

			ProjectCurrentIntensityFrame(pointsOut->data, intensityOut->data,
										 intensityIn->intensity_current, depthIn->depth,
										 intrinsics_depth, intrinsics_rgb,
										 view->calib.trafo_rgb_to_depth.calib_inv);
		}
	}

	// Create raycasted pyramid
	if (useDepth)
	{
		for (int i = 1; i < sceneHierarchy->GetNoLevels(); i++)
		{
			ITMSceneHierarchyLevel *currentLevel = sceneHierarchy->GetLevel(i);
			ITMSceneHierarchyLevel *previousLevel = sceneHierarchy->GetLevel(i - 1);

			lowLevelEngine->FilterSubsampleWithHoles(currentLevel->pointsMap, previousLevel->pointsMap);
			lowLevelEngine->FilterSubsampleWithHoles(currentLevel->normalsMap, previousLevel->normalsMap);
			currentLevel->intrinsics = previousLevel->intrinsics * 0.5f;
		}
	}
}

void ITMExtendedTracker::SetEvaluationParams(int levelId)
{
	currentLevelId = levelId;
	viewHierarchyLevel_Depth = viewHierarchy_Depth->GetLevel(levelId);

	// We use a single sequence of iterationTypes, regardless of the colour/depth choice.
	// Using different strategies for colour and depth does not seem to improve the accuracy (tests by Tommaso, 15/08/2016)
	// We rely on the one stored in viewHierarchyLevel_Depth because it's guaranteed to be available.
	// sceneHierarchyLevel_Depth is available only when depth tracking is enabled
	// viewHierarchyLevel_Intensity is available only when colour is enabled.
	currentIterationType = viewHierarchyLevel_Depth->iterationType;

	if (useDepth)
	{
		// During the optimization, every level of the depth frame pyramid is matched to the full resolution raycast
		sceneHierarchyLevel_Depth = sceneHierarchy->GetLevel(0);
	}

	if (useColour)
	{
		viewHierarchyLevel_Intensity = viewHierarchy_Intensity->GetLevel(levelId);
		reprojectedPointsLevel = reprojectedPointsHierarchy->GetLevel(levelId);
		projectedIntensityLevel = projectedIntensityHierarchy->GetLevel(levelId);
	}
}

void ITMExtendedTracker::ComputeDelta(float *step, float *nabla, float *hessian, bool shortIteration) const
{
	for (int i = 0; i < 6; i++) step[i] = 0;

	if (shortIteration)
	{
		float smallHessian[3 * 3];
		for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) smallHessian[r + c * 3] = hessian[r + c * 6];

		ORUtils::Cholesky cholA(smallHessian, 3);
		cholA.Backsub(step, nabla);
	}
	else
	{
		ORUtils::Cholesky cholA(hessian, 6);
		cholA.Backsub(step, nabla);
	}
}

bool ITMExtendedTracker::HasConverged(float *step) const
{
	for (int i = 0; i < 6; i++)
	{
		if (fabs(step[i]) > terminationThreshold)
		{
			return false;
		}
	}

	return true;
}

void ITMExtendedTracker::ApplyDelta(const Matrix4f & para_old, const float *delta, Matrix4f & para_new) const
{
	float step[6];

	switch (currentIterationType)
	{
	case TRACKER_ITERATION_ROTATION:
		step[0] = (float)(delta[0]); step[1] = (float)(delta[1]); step[2] = (float)(delta[2]);
		step[3] = 0.0f; step[4] = 0.0f; step[5] = 0.0f;
		break;
	case TRACKER_ITERATION_TRANSLATION:
		step[0] = 0.0f; step[1] = 0.0f; step[2] = 0.0f;
		step[3] = (float)(delta[0]); step[4] = (float)(delta[1]); step[5] = (float)(delta[2]);
		break;
	default:
	case TRACKER_ITERATION_BOTH:
		step[0] = (float)(delta[0]); step[1] = (float)(delta[1]); step[2] = (float)(delta[2]);
		step[3] = (float)(delta[3]); step[4] = (float)(delta[4]); step[5] = (float)(delta[5]);
		break;
	}

	Matrix4f Tinc;

	Tinc.m00 = 1.0f;		Tinc.m10 = step[2];		Tinc.m20 = -step[1];	Tinc.m30 = step[3];
	Tinc.m01 = -step[2];	Tinc.m11 = 1.0f;		Tinc.m21 = step[0];		Tinc.m31 = step[4];
	Tinc.m02 = step[1];		Tinc.m12 = -step[0];	Tinc.m22 = 1.0f;		Tinc.m32 = step[5];
	Tinc.m03 = 0.0f;		Tinc.m13 = 0.0f;		Tinc.m23 = 0.0f;		Tinc.m33 = 1.0f;

	para_new = Tinc * para_old;
}

void ITMExtendedTracker::UpdatePoseQuality(int noValidPoints_old, float *hessian_good, float f_old)
{
	if (!useDepth)
	{
		// Currently we cannot handle colour only tracking
		trackingState->trackerResult = ITMTrackingState::TRACKING_GOOD;
		return;
	}

	size_t noTotalPoints = viewHierarchy_Depth->GetLevel(0)->depth->dataSize;
	int noValidPointsMax = lowLevelEngine->CountValidDepths(view->depth);

	float normFactor_v1 = (float)noValidPoints_old / (float)noTotalPoints;
	float normFactor_v2 = (float)noValidPoints_old / (float)noValidPointsMax;

	float det = 0.0f;
	if (currentIterationType == TRACKER_ITERATION_BOTH) {
		ORUtils::Cholesky cholA(hessian_good, 6);
		det = cholA.Determinant();
		if (isnan(det)) det = 0.0f;
	}

	float det_norm_v1 = 0.0f;
	if (currentIterationType == TRACKER_ITERATION_BOTH) {
		float h[6 * 6];
		for (int i = 0; i < 6 * 6; ++i) h[i] = hessian_good[i] * normFactor_v1;
		ORUtils::Cholesky cholA(h, 6);
		det_norm_v1 = cholA.Determinant();
		if (isnan(det_norm_v1)) det_norm_v1 = 0.0f;
	}

	float det_norm_v2 = 0.0f;
	if (currentIterationType == TRACKER_ITERATION_BOTH) {
		float h[6 * 6];
		for (int i = 0; i < 6 * 6; ++i) h[i] = hessian_good[i] * normFactor_v2;
		ORUtils::Cholesky cholA(h, 6);
		det_norm_v2 = cholA.Determinant();
		if (isnan(det_norm_v2)) det_norm_v2 = 0.0f;
	}

	float finalResidual_v2 = sqrt(((float)noValidPoints_old * f_old + (float)(noValidPointsMax - noValidPoints_old) * spaceThresh[0]) / (float)noValidPointsMax);
	float percentageInliers_v2 = (float)noValidPoints_old / (float)noValidPointsMax;

	trackingState->trackerResult = ITMTrackingState::TRACKING_FAILED;
	trackingState->trackerScore = finalResidual_v2;

	if (noValidPointsMax != 0 && noTotalPoints != 0 && det_norm_v1 > 0 && det_norm_v2 > 0) {
		Vector4f inputVector(log(det_norm_v1), log(det_norm_v2), finalResidual_v2, percentageInliers_v2);

		Vector4f normalisedVector = (inputVector - mu) / sigma;

		float mapped[20];
		map->evaluate(mapped, normalisedVector.v, 4);

		float score = svmClassifier->Classify(mapped);

		if (score > 0) trackingState->trackerResult = ITMTrackingState::TRACKING_GOOD;
		else if (score > -10.0f) trackingState->trackerResult = ITMTrackingState::TRACKING_POOR;
	}
}

void ITMExtendedTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
	if (!trackingState->HasValidPointCloud()) return;

	if (trackingState->age_pointCloud >= 0) trackingState->framesProcessed++;
	else trackingState->framesProcessed = 0;

	this->SetEvaluationData(trackingState, view);
	this->PrepareForEvaluation();

	float hessian_good[6 * 6];
	float nabla_good[6];

	for (int i = 0; i < 6 * 6; ++i) hessian_good[i] = 0.0f;
	for (int i = 0; i < 6; ++i) nabla_good[i] = 0.0f;

	// As a workaround to the fact that the SVM has not been updated to handle the Intensity+Depth tracking
	// cache the last depth results and use them when evaluating the tracking quality
	float hessian_depth_good[6 * 6];
	float f_depth_good = 0;
	int noValidPoints_depth_good = 0;
	memset(hessian_depth_good, 0, sizeof(hessian_depth_good));

	for (int levelId = viewHierarchy_Depth->GetNoLevels() - 1; levelId >= 0; levelId--)
	{
		SetEvaluationParams(levelId);

		if (currentIterationType == TRACKER_ITERATION_NONE) continue;

		Matrix4f approxInvPose = trackingState->pose_d->GetInvM();
		ORUtils::SE3Pose lastKnownGoodPose(*(trackingState->pose_d));

		float f_old = std::numeric_limits<float>::max();
		float lambda = 1.0;

		for (int iterNo = 0; iterNo < noIterationsPerLevel[levelId]; iterNo++)
		{
			float hessian_depth[6 * 6], hessian_RGB[6 * 6];
			float nabla_depth[6], nabla_RGB[6];
			float f_depth = 0.f, f_RGB = 0.f;
			int noValidPoints_depth = 0, noValidPoints_RGB = 0;

			// Reset arrays
			memset(hessian_depth, 0, sizeof(hessian_depth));
			memset(hessian_RGB, 0, sizeof(hessian_RGB));

			memset(nabla_depth, 0, sizeof(nabla_depth));
			memset(nabla_RGB, 0, sizeof(nabla_RGB));

			// evaluate error function and gradients
			if (useDepth)
			{
				noValidPoints_depth = ComputeGandH_Depth(f_depth, nabla_depth, hessian_depth, approxInvPose);

				if (noValidPoints_depth > MIN_VALID_POINTS_DEPTH)
				{
					// Normalize nabla and hessian
					for (int i = 0; i < 6 * 6; ++i) hessian_depth[i] /= noValidPoints_depth;
					for (int i = 0; i < 6; ++i) nabla_depth[i] /= noValidPoints_depth;
					f_depth /= noValidPoints_depth;
				}
				else
				{
					f_depth = std::numeric_limits<float>::max();
				}
			}

			if (useColour)
			{
				noValidPoints_RGB = ComputeGandH_RGB(f_RGB, nabla_RGB, hessian_RGB, approxInvPose);

				if (noValidPoints_RGB > MIN_VALID_POINTS_DEPTH)
				{
					// Normalize nabla and hessian
					for (int i = 0; i < 6 * 6; ++i) hessian_RGB[i] /= noValidPoints_RGB;
					for (int i = 0; i < 6; ++i) nabla_RGB[i] /= noValidPoints_RGB;
					f_RGB /= noValidPoints_RGB;
				}
				else
				{
					f_RGB = std::numeric_limits<float>::max();
				}
			}

			float hessian_new[6 * 6];
			float nabla_new[6];
			float f_new = 0.f;
			int noValidPoints_new = 0;

			if (useDepth && useColour)
			{
				// Combine depth and intensity measurements
				if (noValidPoints_depth > MIN_VALID_POINTS_DEPTH)
				{
					noValidPoints_new = noValidPoints_depth;
					f_new = f_depth;
					memcpy(nabla_new, nabla_depth, sizeof(nabla_depth));
					memcpy(hessian_new, hessian_depth, sizeof(hessian_depth));
				}
				else
				{
					// Not enough valid depth correspondences, rely only on colour.
					noValidPoints_new = 0;
					f_new = 0.f;
					memset(nabla_new, 0, sizeof(nabla_new));
					memset(hessian_new, 0, sizeof(hessian_new));
				}

				if (noValidPoints_RGB > MIN_VALID_POINTS_RGB)
				{
					noValidPoints_new += noValidPoints_RGB;
					f_new += f_RGB;
					for (int i = 0; i < 6; ++i) nabla_new[i] += colourWeight * nabla_RGB[i];
					for (int i = 0; i < 6 * 6; ++i) hessian_new[i] += colourWeight * colourWeight * hessian_RGB[i];
				}
			}
			else if (useDepth)
			{
				noValidPoints_new = noValidPoints_depth;
				f_new = f_depth;
				memcpy(nabla_new, nabla_depth, sizeof(nabla_depth));
				memcpy(hessian_new, hessian_depth, sizeof(hessian_depth));
			}
			else if (useColour)
			{
				noValidPoints_new = noValidPoints_RGB;
				f_new = f_RGB;
				memcpy(nabla_new, nabla_RGB, sizeof(nabla_RGB));
				memcpy(hessian_new, hessian_RGB, sizeof(hessian_RGB));
			}
			else
			{
				throw std::runtime_error("Cannot track the camera when both useDepth and useColour are false.");
			}

			// check if error increased. If so, revert
			if ((noValidPoints_new <= 0) || (f_new >= f_old))
			{
				trackingState->pose_d->SetFrom(&lastKnownGoodPose);
				approxInvPose = trackingState->pose_d->GetInvM();
				lambda *= 10.0f;
			}
			else
			{
				lastKnownGoodPose.SetFrom(trackingState->pose_d);
				f_old = f_new;

				for (int i = 0; i < 6 * 6; ++i) hessian_good[i] = hessian_new[i];
				for (int i = 0; i < 6; ++i) nabla_good[i] = nabla_new[i];
				lambda /= 10.0f;

				// Also cache depth results
				noValidPoints_depth_good = noValidPoints_depth;
				f_depth_good = f_depth;
				memcpy(hessian_depth_good, hessian_depth, sizeof(hessian_depth));
			}

			float A[6 * 6];
			for (int i = 0; i < 6 * 6; ++i) A[i] = hessian_good[i];
			for (int i = 0; i < 6; ++i) A[i + i * 6] *= 1.0f + lambda;

			// compute a new step and make sure we've got an SE3
			float step[6];
			ComputeDelta(step, nabla_good, A, currentIterationType != TRACKER_ITERATION_BOTH);

			ApplyDelta(approxInvPose, step, approxInvPose);
			trackingState->pose_d->SetInvM(approxInvPose);
			trackingState->pose_d->Coerce();
			approxInvPose = trackingState->pose_d->GetInvM();

			// if step is small, assume it's going to decrease the error and finish
			if (HasConverged(step)) break;
		}
	}

	this->UpdatePoseQuality(noValidPoints_depth_good, hessian_depth_good, f_depth_good);
}
