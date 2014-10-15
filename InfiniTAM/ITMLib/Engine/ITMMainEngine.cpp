// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMMainEngine.h"

using namespace ITMLib::Engine;

ITMMainEngine::ITMMainEngine(const ITMLibSettings *settings, const ITMRGBDCalib *calib, Vector2i imgSize)
{
	this->settings = new ITMLibSettings(*settings);

	this->scene = new ITMScene<ITMVoxel,ITMVoxelIndex>(&(settings->sceneParams), settings->useSwapping, settings->useGPU);

	this->trackingState = new ITMTrackingState(imgSize, settings->useGPU);
	trackingState->pose_d->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); 

	this->view = new ITMView(*calib, imgSize, settings->useGPU);

	if (settings->useGPU)
	{
#ifndef COMPILE_WITHOUT_CUDA
		lowLevelEngine = new ITMLowLevelEngine_CUDA();

		sceneRecoEngine = new ITMSceneReconstructionEngine_CUDA<ITMVoxel,ITMVoxelIndex>();

		if (settings->trackerType == ITMLibSettings::TRACKER_ICP)
			tracker = new ITMDepthTracker_CUDA(imgSize, settings->noHierarchyLevels, settings->noRotationOnlyLevels, settings->depthTrackerICPThreshold, lowLevelEngine);
		else tracker = new ITMColorTracker_CUDA(imgSize, settings->noHierarchyLevels, settings->noRotationOnlyLevels, lowLevelEngine);

		if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CUDA<ITMVoxel,ITMVoxelIndex>();
		visualisationEngine = new ITMVisualisationEngine_CUDA<ITMVoxel,ITMVoxelIndex>();
#endif
	}
	else
	{
		lowLevelEngine = new ITMLowLevelEngine_CPU();

		sceneRecoEngine = new ITMSceneReconstructionEngine_CPU<ITMVoxel,ITMVoxelIndex>();

		if (settings->trackerType == ITMLibSettings::TRACKER_ICP)
			tracker = new ITMDepthTracker_CPU(imgSize, settings->noHierarchyLevels, settings->noRotationOnlyLevels, settings->depthTrackerICPThreshold, lowLevelEngine);
		else tracker = new ITMColorTracker_CPU(imgSize, settings->noHierarchyLevels, settings->noRotationOnlyLevels, lowLevelEngine);

		if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CPU<ITMVoxel,ITMVoxelIndex>();
		visualisationEngine = new ITMVisualisationEngine_CPU<ITMVoxel,ITMVoxelIndex>();
	}

	visualisationState = NULL;

	hasStartedObjectReconstruction = false;
}

ITMMainEngine::~ITMMainEngine()
{
	delete sceneRecoEngine;
	delete tracker;
	delete lowLevelEngine;

	if (settings->useSwapping) delete swappingEngine;

	delete trackingState;
	delete scene;
	delete view;

	delete settings;
}

void ITMMainEngine::ProcessFrame(void)
{
	bool useGPU = settings->useGPU;
	bool useSwapping = settings->useSwapping;
	bool skipPoints = settings->skipPoints;
	bool useTrackerICP = settings->trackerType == ITMLibSettings::TRACKER_ICP;

	// prepare image and move it to GPU, if required
	if (useGPU)
	{
		view->rgb->UpdateDeviceFromHost();

		switch (view->inputImageType)
		{
		case ITMView::InfiniTAM_FLOAT_DEPTH_IMAGE: view->depth->UpdateDeviceFromHost(); break;
		case ITMView::InfiniTAM_DISPARITY_IMAGE: view->rawDepth->UpdateDeviceFromHost(); break;
		}
	}

	// prepare image and turn it into a depth image
	if (view->inputImageType == ITMView::InfiniTAM_DISPARITY_IMAGE)
		lowLevelEngine->ConvertDisparityToDepth(view->depth, view->rawDepth, &(view->calib->intrinsics_d), &(view->calib->disparityCalib));

	// tracking
	if (hasStartedObjectReconstruction) tracker->TrackCamera(trackingState, view);

	// allocation
	sceneRecoEngine->AllocateSceneFromDepth(scene, view, trackingState->pose_d);

	// integration
	sceneRecoEngine->IntegrateIntoScene(scene, view, trackingState->pose_d);

	if (useSwapping) {
		// swapping: CPU -> GPU
		swappingEngine->IntegrateGlobalIntoLocal(scene, view);
		// swapping: GPU -> CPU
		swappingEngine->SaveToGlobalMemory(scene, view);
	}

	if (useTrackerICP)
	{
		// raycasting
		visualisationEngine->CreateExpectedDepths(scene, trackingState->pose_d, &(view->calib->intrinsics_d), trackingState->renderingRangeImage);
		visualisationEngine->CreateICPMaps(scene, view, trackingState);
	}
	else
	{
		// raycasting
		ITMPose pose_rgb(view->calib->trafo_rgb_to_depth.calib_inv * trackingState->pose_d->M);
		visualisationEngine->CreateExpectedDepths(scene, &pose_rgb, &(view->calib->intrinsics_rgb), trackingState->renderingRangeImage);
		visualisationEngine->CreatePointCloud(scene, view, trackingState, skipPoints);
	}

	hasStartedObjectReconstruction = true;
}

void ITMMainEngine::GetImage(ITMUChar4Image *out, GetImageType getImageType, ITMPose *pose, ITMIntrinsics *intrinsics)
{
	out->Clear();

	switch (getImageType)
	{
	case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
		if (settings->useGPU) view->rgb->UpdateHostFromDevice();
		out->SetFrom(view->rgb);
		break;
	case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
		if (settings->useGPU) view->depth->UpdateHostFromDevice();
		ITMVisualisationEngine<ITMVoxel,ITMVoxelIndex>::DepthToUchar4(out, view->depth);
		break;
	case ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST:
		if (settings->useGPU) trackingState->rendering->UpdateHostFromDevice();
		out->SetFrom(trackingState->rendering);
		break;
	case ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST_FREECAMERA:
		if (pose != NULL && intrinsics != NULL)
		{
			if (visualisationState == NULL) visualisationState = visualisationEngine->allocateInternalState(out->noDims);

			visualisationEngine->FindVisibleBlocks(scene, pose, intrinsics, visualisationState);
			visualisationEngine->CreateExpectedDepths(scene, pose, intrinsics, visualisationState->minmaxImage, visualisationState);
			visualisationEngine->RenderImage(scene, pose, intrinsics, visualisationState, visualisationState->outputImage);

			if (settings->useGPU) visualisationState->outputImage->UpdateHostFromDevice();
			out->SetFrom(visualisationState->outputImage);
		}
		break;
	};
}