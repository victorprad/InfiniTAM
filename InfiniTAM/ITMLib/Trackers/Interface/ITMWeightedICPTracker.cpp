// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMWeightedICPTracker.h"
#include "../../../ORUtils/Cholesky.h"

#include <math.h>

using namespace ITMLib;

ITMWeightedICPTracker::ITMWeightedICPTracker(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
	float terminationThreshold, const ITMLowLevelEngine *lowLevelEngine, MemoryDeviceType memoryType)
{
	if (memoryType==MEMORYDEVICE_CUDA) 
	{
		weightHierarchy = new ITMImageHierarchy<ITMTemplatedHierarchyLevel<ITMFloatImage> >(imgSize, trackingRegime, noHierarchyLevels, memoryType, true);
		viewHierarchy = new ITMImageHierarchy<ITMTemplatedHierarchyLevel<ITMFloatImage> >(imgSize, trackingRegime, noHierarchyLevels, memoryType, true);
		sceneHierarchy = new ITMImageHierarchy<ITMSceneHierarchyLevel>(imgSize, trackingRegime, noHierarchyLevels, memoryType, true);
	}
	else
	{
		weightHierarchy = new ITMImageHierarchy<ITMTemplatedHierarchyLevel<ITMFloatImage> >(imgSize, trackingRegime, noHierarchyLevels, memoryType, true);
		viewHierarchy = new ITMImageHierarchy<ITMTemplatedHierarchyLevel<ITMFloatImage> >(imgSize, trackingRegime, noHierarchyLevels, memoryType, true);
		sceneHierarchy = new ITMImageHierarchy<ITMSceneHierarchyLevel>(imgSize, trackingRegime, noHierarchyLevels, memoryType, true);
	}

	this->noIterationsPerLevel = new int[noHierarchyLevels];
	this->distThresh = new float[noHierarchyLevels];
	
	this->noIterationsPerLevel[0] = 2; //TODO -> make parameter
	for (int levelId = 1; levelId < noHierarchyLevels; levelId++)
	{
		noIterationsPerLevel[levelId] = noIterationsPerLevel[levelId - 1] + 2;
	}

	float distThreshStep = distThresh / noHierarchyLevels;
	this->distThresh[noHierarchyLevels - 1] = distThresh;
	for (int levelId = noHierarchyLevels - 2; levelId >= 0; levelId--)
		this->distThresh[levelId] = this->distThresh[levelId + 1] - distThreshStep;

	this->lowLevelEngine = lowLevelEngine;

	this->noICPLevel = noICPRunTillLevel;

	this->terminationThreshold = terminationThreshold;

	map = new ORUtils::HomkerMap(2);
	svmClassifier = new ORUtils::SVMClassifier(map->getDescriptorSize(4));

	//all below obtained from dataset in matlab
	float w[20];
	w[0] = -3.15813f; w[1] = -2.38038f; w[2] = 1.93359f; w[3] = 1.56642f; w[4] = 1.76306f;
	w[5] = -0.747641f; w[6] = 4.41852f; w[7] = 1.72048f; w[8] = -0.482545f; w[9] = -5.07793f;
	w[10] = 1.98676f; w[11] = -0.45688f; w[12] = 2.53969f; w[13] = -3.50527f; w[14] = -1.68725f;
	w[15] = 2.31608f; w[16] = 5.14778f; w[17] = 2.31334f; w[18] = -14.128f; w[19] = 6.76423f;

	float b = 9.334260e-01f + 2.5f; //2.5f is manual threshold on bias

	mu = Vector4f(-34.9470512137603f, -33.1379108518478f, 0.195948598235857f, 0.611027292662361f);
	sigma = Vector4f(68.1654461020426f, 60.6607826748643f, 0.00343068557187040f, 0.0402595570918749f);

	svmClassifier->SetVectors(w, b);
}

ITMWeightedICPTracker::~ITMWeightedICPTracker(void)
{ 
	delete this->viewHierarchy;
	delete this->weightHierarchy;
	delete this->sceneHierarchy;
	

	delete[] this->noIterationsPerLevel;
}

void ITMWeightedICPTracker::SetEvaluationData(ITMTrackingState *trackingState, const ITMView *view)
{
	this->trackingState = trackingState;
	this->view = view;

	sceneHierarchy->levels[0]->intrinsics = view->calib->intrinsics_d.projectionParamsSimple.all;
	viewHierarchy->levels[0]->intrinsics = view->calib->intrinsics_d.projectionParamsSimple.all;

	// the image hierarchy allows pointers to external data at level 0
	viewHierarchy->levels[0]->depth = view->depth;
	weightHierarchy->levels[0]->depth = view->depthUncertainty;
	
	sceneHierarchy->levels[0]->pointsMap = trackingState->pointCloud->locations;
	sceneHierarchy->levels[0]->normalsMap = trackingState->pointCloud->colours;

	scenePose = trackingState->pose_pointCloud->GetM();
}

void ITMWeightedICPTracker::PrepareForEvaluation()
{
	for (int i = 1; i < viewHierarchy->noLevels; i++)
	{
		ITMTemplatedHierarchyLevel<ITMFloatImage> *currentWICPLevel = viewHierarchy->levels[i], *previousWICPHierarch = viewHierarchy->levels[i - 1];
		lowLevelEngine->FilterSubsampleWithHoles(currentWICPLevel->depth, previousWICPHierarch->depth);
		currentWICPLevel->intrinsics = previousWICPHierarch->intrinsics * 0.5f;

		currentWICPLevel = weightHierarchy->levels[i], previousWICPHierarch = weightHierarchy->levels[i - 1];
		lowLevelEngine->FilterSubsampleWithHoles(currentWICPLevel->depth, previousWICPHierarch->depth);

		ITMSceneHierarchyLevel *currentLevelScene = sceneHierarchy->levels[i], *previousLevelScene = sceneHierarchy->levels[i - 1];
		currentLevelScene->intrinsics = previousLevelScene->intrinsics * 0.5f;
	}
}

void ITMWeightedICPTracker::SetEvaluationParams(int levelId)
{
	this->levelId = levelId;
	this->iterationType = viewHierarchy->levels[levelId]->iterationType;
	this->sceneHierarchyLevel = sceneHierarchy->levels[0];
	this->viewHierarchyLevel = viewHierarchy->levels[levelId];
	this->weightHierarchyLevel = weightHierarchy->levels[levelId];
}

void ITMWeightedICPTracker::ComputeDelta(float *step, float *nabla, float *hessian, bool shortIteration) const
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

bool ITMWeightedICPTracker::HasConverged(float *step) const
{

	float stepLength = 0.0f;
	for (int i = 0; i < 6; i++) stepLength += step[i] * step[i];

	if (sqrt(stepLength) / 6 < terminationThreshold) return true; //converged

	return false;
}

void ITMWeightedICPTracker::ApplyDelta(const Matrix4f & para_old, const float *delta, Matrix4f & para_new) const
{
	float step[6];

	switch (iterationType)
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


void ITMWeightedICPTracker::UpdatePoseQuality(int noValidPoints_old, float *hessian_good, float f_old)
{
	int noTotalPoints = viewHierarchy->levels[0]->depth->noDims.x * viewHierarchy->levels[0]->depth->noDims.y;

	int noValidPointsMax = lowLevelEngine->CountValidDepths(view->depth);

	float normFactor_v1 = (float)noValidPoints_old / (float)noTotalPoints;
	float normFactor_v2 = (float)noValidPoints_old / (float)noValidPointsMax;

	float det = 0.0f;
	if (iterationType == TRACKER_ITERATION_BOTH) {
		ORUtils::Cholesky cholA(hessian_good, 6);
		det = cholA.Determinant();
		if (isnan(det)) det = 0.0f;
	}

	float det_norm_v1 = 0.0f;
	if (iterationType == TRACKER_ITERATION_BOTH) {
		float h[6 * 6];
		for (int i = 0; i < 6 * 6; ++i) h[i] = hessian_good[i] * normFactor_v1;
		ORUtils::Cholesky cholA(h, 6);
		det_norm_v1 = cholA.Determinant();
		if (isnan(det_norm_v1)) det_norm_v1 = 0.0f;
	}

	float det_norm_v2 = 0.0f;
	if (iterationType == TRACKER_ITERATION_BOTH) {
		float h[6 * 6];
		for (int i = 0; i < 6 * 6; ++i) h[i] = hessian_good[i] * normFactor_v2;
		ORUtils::Cholesky cholA(h, 6);
		det_norm_v2 = cholA.Determinant();
		if (isnan(det_norm_v2)) det_norm_v2 = 0.0f;
	}

	float finalResidual_v2 = sqrt(((float)noValidPoints_old * f_old + (float)(noValidPointsMax - noValidPoints_old) * distThresh[0]) / (float)noValidPointsMax);
	float percentageInliers_v2 = (float)noValidPoints_old / (float)noValidPointsMax;

	if (noValidPointsMax != 0 && noTotalPoints != 0 && det_norm_v1 > 0 && det_norm_v2 > 0) {
		Vector4f inputVector(log(det_norm_v1), log(det_norm_v2), finalResidual_v2, percentageInliers_v2);

		Vector4f normalisedVector = (inputVector - mu) / sigma;

		float mapped[20];
		map->evaluate(mapped, normalisedVector.v, 4);

		float score = svmClassifier->Classify(mapped);

		if (score > 0) trackingState->poseQuality = 1.0f;
		else if (score > -10.0f) trackingState->poseQuality = 0.5f;
		else trackingState->poseQuality = 0.2f;

		printf("score: %f\n", score);
	}
}

void ITMWeightedICPTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
	this->SetEvaluationData(trackingState, view);
	this->PrepareForEvaluation();

	Matrix4f approxInvPose = trackingState->pose_d->GetInvM();

	float f_old = 1e10, f_new;

	int noValidPoints = 0;

	for (int levelId = viewHierarchy->noLevels - 1; levelId >= noICPLevel; levelId--)
	{
		this->SetEvaluationParams(levelId);

		if (iterationType == TRACKER_ITERATION_NONE) continue;

		for (int iterNo = 0; iterNo < noIterationsPerLevel[levelId]; iterNo++)
		{
			noValidPoints = this->ComputeGandH(f_new, nabla, hessian, approxInvPose);

			if (noValidPoints <= 0) break;
			if (f_new > f_old) break;

			ComputeDelta(step, nabla, hessian, iterationType != TRACKER_ITERATION_BOTH);
			ApplyDelta(approxInvPose, step, approxInvPose);
			trackingState->pose_d->SetInvM(approxInvPose);
			trackingState->pose_d->Coerce();
			approxInvPose = trackingState->pose_d->GetInvM();
			if (HasConverged(step)) break;
		}
	}

	//// not working
	//this->UpdatePoseQuality(noValidPoints, hessian, f_old);
}

