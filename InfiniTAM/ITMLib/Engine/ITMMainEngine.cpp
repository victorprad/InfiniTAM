// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMMainEngine.h"

#include "ITMTrackerFactory.h"
using namespace ITMLib::Engine;

ITMMainEngine::ITMMainEngine(const ITMLibSettings *settings, const ITMRGBDCalib *calib, Vector2i imgSize_rgb, Vector2i imgSize_d)
{
	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

	this->settings = new ITMLibSettings(*settings);

	this->trackingState = ITMTrackerFactory::MakeTrackingState(*settings, imgSize_rgb, imgSize_d);
	trackingState->pose_d->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); 

	this->view = new ITMView();

	denseMapper = new ITMDenseMapper<ITMVoxel, ITMVoxelIndex>(this->settings, imgSize_rgb, imgSize_d);

	switch (settings->deviceType)
	{
	case ITMLibSettings::DEVICE_CPU:
		lowLevelEngine = new ITMLowLevelEngine_CPU();
		viewBuilder = new ITMViewBuilder_CPU(calib, settings->deviceType);
		break;
	case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
		lowLevelEngine = new ITMLowLevelEngine_CUDA();
		viewBuilder = new ITMViewBuilder_CUDA(calib, settings->deviceType);
#endif
		break;
	case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
		lowLevelEngine = new ITMLowLevelEngine_Metal();
		viewBuilder = new ITMViewBuilder_Metal(calib, settings->deviceType);
#endif
		break;
	}

	this->trackerPrimary = ITMTrackerFactory::MakePrimaryTracker(*settings, imgSize_rgb, imgSize_d, lowLevelEngine);
	this->trackerSecondary = ITMTrackerFactory::MakeSecondaryTracker<ITMVoxel, ITMVoxelIndex>(*settings, imgSize_rgb, imgSize_d, lowLevelEngine, denseMapper->getScene());

	hasStartedObjectReconstruction = false;
	fusionActive = true;
	mainProcessingActive = true;
}

ITMMainEngine::~ITMMainEngine()
{
	delete denseMapper;

	if (trackerPrimary != NULL) delete trackerPrimary;
	if (trackerSecondary != NULL) delete trackerSecondary;

	delete lowLevelEngine;
	delete viewBuilder;

	delete trackingState;
	delete view;

	delete settings;
}

void ITMMainEngine::ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
	// prepare image and turn it into a depth image
	viewBuilder->UpdateView(view, rgbImage, rawDepthImage);

	if (!mainProcessingActive) return;

	// tracking
	if (hasStartedObjectReconstruction)
	{
		if (trackerPrimary != NULL) trackerPrimary->TrackCamera(trackingState, view);
		if (trackerSecondary != NULL) trackerSecondary->TrackCamera(trackingState, view);
	}

	if (fusionActive) denseMapper->ProcessFrame(view, trackingState);

	switch (settings->trackerType)
	{
	case ITMLibSettings::TRACKER_ICP:
	case ITMLibSettings::TRACKER_REN:
		// raycasting
		denseMapper->GetICPMaps(trackingState->pose_d, &(view->calib->intrinsics_d), view, trackingState);
		break;
	case ITMLibSettings::TRACKER_COLOR:
		// raycasting
		ITMPose pose_rgb(view->calib->trafo_rgb_to_depth.calib_inv * trackingState->pose_d->M);
		denseMapper->GetPointCloud(&pose_rgb, &(view->calib->intrinsics_rgb), view, trackingState, settings->skipPoints);
		break;
	}

	hasStartedObjectReconstruction = true;
}

void ITMMainEngine::GetImage(ITMUChar4Image *out, GetImageType getImageType, bool useColour, ITMPose *pose, ITMIntrinsics *intrinsics)
{
	if (!view->isAllocated) return;

	out->Clear();

	switch (getImageType)
	{
	case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->rgb->UpdateHostFromDevice();
		out->ChangeDims(view->rgb->noDims);
		out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depth->UpdateHostFromDevice();
		out->ChangeDims(view->depth->noDims);
		ITMVisualisationEngine<ITMVoxel,ITMVoxelIndex>::DepthToUchar4(out, view->depth);
		break;
	case ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST:
	{
		ORUtils::Image<Vector4u> *srcImage = denseMapper->renderState_live->raycastImage;
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) srcImage->UpdateHostFromDevice();
		out->ChangeDims(srcImage->noDims);
		out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	}
	case ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST_FREECAMERA:
		return denseMapper->GetRendering(pose, intrinsics, useColour, out);
		break;
	};
}

void ITMMainEngine::turnOnIntegration() { fusionActive = true; }
void ITMMainEngine::turnOffIntegration() { fusionActive = false; }
void ITMMainEngine::turnOnMainProcessing() { mainProcessingActive = true; }
void ITMMainEngine::turnOffMainProcessing() { mainProcessingActive = false; }
