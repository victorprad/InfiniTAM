// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMMainEngine.h"

#include "ITMTrackerFactory.h"
using namespace ITMLib::Engine;

ITMMainEngine::ITMMainEngine(const ITMLibSettings *settings, const ITMRGBDCalib *calib, Vector2i imgSize_rgb, Vector2i imgSize_d)
{
	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

	this->settings = new ITMLibSettings(*settings);

	this->scene = new ITMScene<ITMVoxel,ITMVoxelIndex>(&(settings->sceneParams), settings->useSwapping, settings->deviceType == ITMLibSettings::DEVICE_CUDA);

	this->trackingState = ITMTrackerFactory::MakeTrackingState(*settings, imgSize_rgb, imgSize_d);
	trackingState->pose_d->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); 

	this->view = new ITMView(*calib, imgSize_rgb, imgSize_d, settings->deviceType == ITMLibSettings::DEVICE_CUDA);

	switch (settings->deviceType)
	{
	case ITMLibSettings::DEVICE_CPU:
		lowLevelEngine = new ITMLowLevelEngine_CPU();
		sceneRecoEngine = new ITMSceneReconstructionEngine_CPU<ITMVoxel, ITMVoxelIndex>();
		if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CPU<ITMVoxel, ITMVoxelIndex>();
		visualisationEngine = new ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>();
		break;
	case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
		lowLevelEngine = new ITMLowLevelEngine_CUDA();
		sceneRecoEngine = new ITMSceneReconstructionEngine_CUDA<ITMVoxel, ITMVoxelIndex>();
		if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CUDA<ITMVoxel, ITMVoxelIndex>();
		visualisationEngine = new ITMVisualisationEngine_CUDA<ITMVoxel, ITMVoxelIndex>();
#endif
		break;
	case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
        lowLevelEngine = new ITMLowLevelEngine_Metal();
        sceneRecoEngine = new ITMSceneReconstructionEngine_Metal<ITMVoxel, ITMVoxelIndex>();
        if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CPU<ITMVoxel, ITMVoxelIndex>();
        visualisationEngine = new ITMVisualisationEngine_Metal<ITMVoxel, ITMVoxelIndex>();
#endif
		break;
	}

	this->renderState_live = visualisationEngine->CreateRenderState(scene, ITMTrackerFactory::GetTrackedImageSize(*settings, imgSize_rgb, imgSize_d));

	this->trackerPrimary = ITMTrackerFactory::MakePrimaryTracker(*settings, imgSize_rgb, imgSize_d, lowLevelEngine);
	this->trackerSecondary = ITMTrackerFactory::MakeSecondaryTracker<ITMVoxel, ITMVoxelIndex>(*settings, imgSize_rgb, imgSize_d, lowLevelEngine, scene);

	this->renderState_freeview = NULL;

	hasStartedObjectReconstruction = false;
	fusionActive = true;
}

ITMMainEngine::~ITMMainEngine()
{
	delete sceneRecoEngine;
	if (trackerPrimary != NULL) delete trackerPrimary;
	if (trackerSecondary != NULL) delete trackerSecondary;
	delete visualisationEngine;
	delete lowLevelEngine;

	if (settings->useSwapping) delete swappingEngine;

	delete trackingState;
	delete renderState_live;

	delete scene;
	delete view;

	if (renderState_freeview != NULL) delete renderState_freeview;

	delete settings;
}

void ITMMainEngine::ProcessFrame(void)
{
	bool useSwapping = settings->useSwapping;

	// prepare image and move it to GPU, if required
	if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
	{
		view->rgb->UpdateDeviceFromHost();

		switch (view->inputImageType)
		{
		case ITMView::InfiniTAM_FLOAT_DEPTH_IMAGE: view->depth->UpdateDeviceFromHost(); break;
		case ITMView::InfiniTAM_SHORT_DEPTH_IMAGE:
		case ITMView::InfiniTAM_DISPARITY_IMAGE: view->rawDepth->UpdateDeviceFromHost(); break;
		}
	}

	// prepare image and turn it into a depth image
	if (view->inputImageType == ITMView::InfiniTAM_DISPARITY_IMAGE)
		lowLevelEngine->ConvertDisparityToDepth(view->depth, view->rawDepth, &(view->calib->intrinsics_d), &(view->calib->disparityCalib));
	else if (view->inputImageType == ITMView::InfiniTAM_SHORT_DEPTH_IMAGE)
		lowLevelEngine->ConvertDepthMMToFloat(view->depth, view->rawDepth);

	// tracking
	if (hasStartedObjectReconstruction)
	{
		if (trackerPrimary != NULL) trackerPrimary->TrackCamera(trackingState, view);
		if (trackerSecondary != NULL) trackerSecondary->TrackCamera(trackingState, view);
	}

	// allocation
	sceneRecoEngine->AllocateSceneFromDepth(scene, view, trackingState, renderState_live);

	// integration
	if (fusionActive) sceneRecoEngine->IntegrateIntoScene(scene, view, trackingState, renderState_live);

	if (useSwapping) {
		// swapping: CPU -> GPU
		swappingEngine->IntegrateGlobalIntoLocal(scene, view, renderState_live);
		// swapping: GPU -> CPU
		swappingEngine->SaveToGlobalMemory(scene, view, renderState_live);
	}

	switch (settings->trackerType)
	{
	case ITMLibSettings::TRACKER_ICP:
	case ITMLibSettings::TRACKER_REN:
		// raycasting
		visualisationEngine->CreateExpectedDepths(scene, trackingState->pose_d, &(view->calib->intrinsics_d), renderState_live);
		visualisationEngine->CreateICPMaps(scene, view, trackingState, renderState_live);
		break;
	case ITMLibSettings::TRACKER_COLOR:
		// raycasting
		ITMPose pose_rgb(view->calib->trafo_rgb_to_depth.calib_inv * trackingState->pose_d->M);
		visualisationEngine->CreateExpectedDepths(scene, &pose_rgb, &(view->calib->intrinsics_rgb), renderState_live);
		visualisationEngine->CreatePointCloud(scene, view, trackingState, renderState_live, settings->skipPoints);
		break;
	}

	hasStartedObjectReconstruction = true;
}

void ITMMainEngine::GetImage(ITMUChar4Image *out, GetImageType getImageType, bool useColour, ITMPose *pose, ITMIntrinsics *intrinsics)
{
	out->Clear();

	switch (getImageType)
	{
	case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->rgb->UpdateHostFromDevice();
		out->ChangeDims(view->rgb->noDims);
		out->SetFrom(view->rgb);
		break;
	case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depth->UpdateHostFromDevice();
		out->ChangeDims(view->depth->noDims);
		ITMVisualisationEngine<ITMVoxel,ITMVoxelIndex>::DepthToUchar4(out, view->depth);
		break;
	case ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST:
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) renderState_live->raycastImage->UpdateHostFromDevice();
		out->ChangeDims(renderState_live->raycastImage->noDims);
		out->SetFrom(renderState_live->raycastImage);
		break;
	case ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST_FREECAMERA:
		if (pose != NULL && intrinsics != NULL)
		{
			if (renderState_freeview == NULL) renderState_freeview = visualisationEngine->CreateRenderState(scene, out->noDims);

			visualisationEngine->FindVisibleBlocks(scene, pose, intrinsics, renderState_freeview);
			visualisationEngine->CreateExpectedDepths(scene, pose, intrinsics, renderState_freeview);
			visualisationEngine->RenderImage(scene, pose, intrinsics, renderState_freeview, renderState_freeview->raycastImage, useColour);

			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) renderState_freeview->raycastImage->UpdateHostFromDevice();
			out->SetFrom(renderState_freeview->raycastImage);
		}
		break;
	};
}

void ITMMainEngine::turnOnIntegration()
{
	fusionActive = true;
}

void ITMMainEngine::turnOffIntegration()
{
	fusionActive = false;
}
