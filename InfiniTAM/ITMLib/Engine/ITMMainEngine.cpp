// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMMainEngine.h"

using namespace ITMLib::Engine;

ITMMainEngine::ITMMainEngine(const ITMLibSettings *settings, const ITMRGBDCalib *calib, Vector2i imgSize_rgb, Vector2i imgSize_d)
{
	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

	this->settings = new ITMLibSettings(*settings);

	denseMapper = new ITMDenseMapper<ITMVoxel, ITMVoxelIndex>(this->settings, ITMTrackerCollection::GetTrackedImageSize(settings,
		imgSize_rgb, imgSize_d));

	switch (settings->deviceType)
	{
	case ITMLibSettings::DEVICE_CPU:
		lowLevelEngine = new ITMLowLevelEngine_CPU();
		viewBuilder = new ITMViewBuilder_CPU(calib);
		break;
	case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
		lowLevelEngine = new ITMLowLevelEngine_CUDA();
		viewBuilder = new ITMViewBuilder_CUDA(calib);
#endif
		break;
	case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
		lowLevelEngine = new ITMLowLevelEngine_Metal();
		viewBuilder = new ITMViewBuilder_Metal(calib);
#endif
		break;
	}

	this->trackerCollection = new ITMTrackerCollection(settings, imgSize_rgb, imgSize_d, denseMapper, lowLevelEngine);
	trackingState = trackerCollection->BuildTrackingState();

	this->view = NULL; // will be allocated by the view builder

	hasStartedObjectReconstruction = false;
	fusionActive = true;
	mainProcessingActive = true;
}

ITMMainEngine::~ITMMainEngine()
{
	delete denseMapper;
	delete trackerCollection;

	delete lowLevelEngine;
	delete viewBuilder;

	delete trackingState;
	if (view != NULL) delete view;

	delete settings;
}

void ITMMainEngine::ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
	// prepare image and turn it into a depth image
	viewBuilder->UpdateView(&view, rgbImage, rawDepthImage);

	if (!mainProcessingActive) return;

	// tracking
	if (hasStartedObjectReconstruction) trackerCollection->TrackCamera(trackingState, view);

	// fusion
	if (fusionActive) denseMapper->ProcessFrame(view, trackingState);

	// raycast
	trackerCollection->RenderWorld(denseMapper, trackingState, view);

	hasStartedObjectReconstruction = true;
}

void ITMMainEngine::ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement)
{
	// prepare image and turn it into a depth image
	viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, imuMeasurement);

	if (!mainProcessingActive) return;

	// tracking
	if (hasStartedObjectReconstruction) trackerCollection->TrackCamera(trackingState, view);

	// fusion
	if (fusionActive) denseMapper->ProcessFrame(view, trackingState);

	// raycast
	trackerCollection->RenderWorld(denseMapper, trackingState, view);

	hasStartedObjectReconstruction = true;
}

void ITMMainEngine::GetImage(ITMUChar4Image *out, GetImageType getImageType, bool useColour, ITMPose *pose, ITMIntrinsics *intrinsics)
{
	if (view == NULL) return;

	out->Clear();

	switch (getImageType)
	{
	case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
		out->ChangeDims(view->rgb->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) 
			out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
		out->ChangeDims(view->depth->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depth->UpdateHostFromDevice();
		ITMVisualisationEngine<ITMVoxel,ITMVoxelIndex>::DepthToUchar4(out, view->depth);
		break;
	case ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST:
	{
		ORUtils::Image<Vector4u> *srcImage = denseMapper->renderState_live->raycastImage;
		out->ChangeDims(srcImage->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
			out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);	
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
