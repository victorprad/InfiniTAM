// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDepthTracker.h"
#include "../Utils/ITMCholesky.h"

#include <math.h>

using namespace ITMLib::Engine;
using namespace ITMLib::Utils;

ITMDepthTracker::ITMDepthTracker(Vector2i imgSize, int noHierarchyLevels, int noRotationOnlyLevels, int noICPRunTillLevel, float distThresh, ITMLowLevelEngine *lowLevelEngine, bool useGPU)
{
	viewHierarchy = new ITMImageHierarchy<ITMTemplatedHierarchyLevel<ITMFloatImage> >(imgSize, noHierarchyLevels, noRotationOnlyLevels, useGPU);
	sceneHierarchy = new ITMImageHierarchy<ITMSceneHierarchyLevel>(imgSize, noHierarchyLevels, noRotationOnlyLevels, useGPU);

	this->noIterationsPerLevel = new int[noHierarchyLevels];

	this->noIterationsPerLevel[0] = 2; //TODO -> make parameter
	for (int levelId = 1; levelId < noHierarchyLevels; levelId++)
	{
		noIterationsPerLevel[levelId] = noIterationsPerLevel[levelId - 1] + 2;
	}

	this->lowLevelEngine = lowLevelEngine;

	this->distThresh = distThresh;

	this->noICPLevel = noICPRunTillLevel;
}

ITMDepthTracker::~ITMDepthTracker(void) 
{ 
	delete this->viewHierarchy;
	delete this->sceneHierarchy;

	delete[] this->noIterationsPerLevel;
}

void ITMDepthTracker::SetEvaluationData(ITMTrackingState *trackingState, const ITMView *view)
{
	this->trackingState = trackingState;
	this->view = view;

	sceneHierarchy->levels[0]->intrinsics = view->calib->intrinsics_d.projectionParamsSimple.all;
	viewHierarchy->levels[0]->intrinsics = view->calib->intrinsics_d.projectionParamsSimple.all;

	lowLevelEngine->CopyImage(viewHierarchy->levels[0]->depth, view->depth);
	lowLevelEngine->CopyImage(sceneHierarchy->levels[0]->pointsMap, trackingState->pointCloud->locations);
	lowLevelEngine->CopyImage(sceneHierarchy->levels[0]->normalsMap, trackingState->pointCloud->colours);
}

void ITMDepthTracker::PrepareForEvaluation()
{
	this->ChangeIgnorePixelToZero(viewHierarchy->levels[0]->depth);

	for (int i = 1; i < viewHierarchy->noLevels; i++)
	{
		ITMTemplatedHierarchyLevel<ITMFloatImage> *currentLevelView = viewHierarchy->levels[i], *previousLevelView = viewHierarchy->levels[i - 1];
		lowLevelEngine->FilterSubsampleWithHoles(currentLevelView->depth, previousLevelView->depth);
		currentLevelView->intrinsics = previousLevelView->intrinsics * 0.5f;

		ITMSceneHierarchyLevel *currentLevelScene = sceneHierarchy->levels[i], *previousLevelScene = sceneHierarchy->levels[i - 1];
		//lowLevelEngine->FilterSubsampleWithHoles(currentLevelScene->pointsMap, previousLevelScene->pointsMap);
		//lowLevelEngine->FilterSubsampleWithHoles(currentLevelScene->normalsMap, previousLevelScene->normalsMap);
		currentLevelScene->intrinsics = previousLevelScene->intrinsics * 0.5f;
	}
}

void ITMDepthTracker::SetEvaluationParams(int levelId)
{
	this->levelId = levelId;
	this->rotationOnly = viewHierarchy->levels[levelId]->rotationOnly;
}

void ITMDepthTracker::ComputeSingleStep(float *step, float *ATA, float *ATb, bool rotationOnly)
{
	for (int i = 0; i < 6; i++) step[i] = 0;

	if (rotationOnly)
	{
		float smallATA[3 * 3];
		for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) smallATA[r + c * 3] = ATA[r + c * 6];

		ITMCholesky cholA(smallATA, 3);
		cholA.Backsub(step, ATb);
	}
	else
	{
		ITMCholesky cholA(ATA, 6);
		cholA.Backsub(step, ATb);
	}
}

Matrix4f ITMDepthTracker::ApplySingleStep(Matrix4f approxInvPose, float *step)
{
	Matrix4f Tinc;

	Tinc.m00 = 1.0f;		Tinc.m10 = step[2];		Tinc.m20 = -step[1];	Tinc.m30 = step[3];
	Tinc.m01 = -step[2];	Tinc.m11 = 1.0f;		Tinc.m21 = step[0];		Tinc.m31 = step[4];
	Tinc.m02 = step[1];		Tinc.m12 = -step[0];	Tinc.m22 = 1.0f;		Tinc.m32 = step[5];
	Tinc.m03 = 0.0f;		Tinc.m13 = 0.0f;		Tinc.m23 = 0.0f;		Tinc.m33 = 1.0f;

	return Tinc * approxInvPose;
}

void ITMDepthTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
	this->SetEvaluationData(trackingState, view);

	this->PrepareForEvaluation();

	Matrix4f approxInvPose = trackingState->pose_d->invM, imagePose = trackingState->pose_d->M;

	for (int levelId = viewHierarchy->noLevels - 1; levelId >= noICPLevel; levelId--)
	{
		
		this->SetEvaluationParams(levelId);

		int noValidPoints;

		ITMSceneHierarchyLevel *sceneHierarchyLevel = sceneHierarchy->levels[0];
		ITMTemplatedHierarchyLevel<ITMFloatImage> *viewHierarchyLevel = viewHierarchy->levels[levelId];

		for (int iterNo = 0; iterNo < noIterationsPerLevel[levelId]; iterNo++)
		{
			noValidPoints = this->ComputeGandH(sceneHierarchyLevel, viewHierarchyLevel, approxInvPose, imagePose, rotationOnly);

			if (noValidPoints > 0)
			{
				this->ComputeSingleStep(step, ATA_host, ATb_host, rotationOnly);

				float stepLength = 0.0f;
				for (int i = 0; i < 6; i++) stepLength += step[i] * step[i];

				if (sqrtf(stepLength) / 6 < 1e-3) break; //converged

				approxInvPose = ApplySingleStep(approxInvPose, step);
			}
		}
	}

	approxInvPose.inv(trackingState->pose_d->M);
	trackingState->pose_d->SetRTInvM_FromM();
	trackingState->pose_d->SetParamsFromModelView();
	trackingState->pose_d->SetModelViewFromParams();

	//printf(">> %f %f %f %f %f %f\n", scene->pose->params.each.rx, scene->pose->params.each.ry, scene->pose->params.each.rz,
	//	scene->pose->params.each.tx, scene->pose->params.each.ty, scene->pose->params.each.tz);
}

