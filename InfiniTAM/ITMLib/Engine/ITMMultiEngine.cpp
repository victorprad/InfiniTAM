// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMMultiEngine.h"

using namespace ITMLib;

ITMMultiEngine::ITMMultiEngine(const ITMLibSettings *settings, const ITMRGBDCalib *calib, Vector2i imgSize_rgb, Vector2i imgSize_d)
{
	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

	this->settings = settings;

	switch (settings->deviceType)
	{
	case ITMLibSettings::DEVICE_CPU:
		lowLevelEngine = new ITMLowLevelEngine_CPU();
		viewBuilder = new ITMViewBuilder_CPU(calib);
		visualisationEngine = new ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>();
		break;
	case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
		lowLevelEngine = new ITMLowLevelEngine_CUDA();
		viewBuilder = new ITMViewBuilder_CUDA(calib);
		visualisationEngine = new ITMVisualisationEngine_CUDA<ITMVoxel, ITMVoxelIndex>();
#endif
		break;
	case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
		lowLevelEngine = new ITMLowLevelEngine_Metal();
		viewBuilder = new ITMViewBuilder_Metal(calib);
		visualisationEngine = new ITMVisualisationEngine_Metal<ITMVoxel, ITMVoxelIndex>();
#endif
		break;
	}

	trackedImageSize = ITMTrackingController::GetTrackedImageSize(settings, imgSize_rgb, imgSize_d);

	renderState_freeview = NULL; //will be created by the visualisation engine

	denseMapper = new ITMDenseMapper<ITMVoxel, ITMVoxelIndex>(settings);

	imuCalibrator = new ITMIMUCalibrator_iPad();
	tracker = ITMTrackerFactory<ITMVoxel, ITMVoxelIndex>::Instance().Make(trackedImageSize, settings, lowLevelEngine, imuCalibrator, NULL/*scene TODO: this will fail for Ren Tracker*/);
	trackingController = new ITMTrackingController(tracker, visualisationEngine, lowLevelEngine, settings);

//	trackingState = trackingController->BuildTrackingState(trackedImageSize);
	allData.push_back(new ITMLocalModelData<ITMVoxel,ITMVoxelIndex>(settings, visualisationEngine, trackingController, trackedImageSize));
	activeDataIdx.push_back(0);
	primaryDataIdx = 0;

	tracker->UpdateInitialPose(allData[primaryDataIdx]->trackingState);

	view = NULL; // will be allocated by the view builder
}

ITMMultiEngine::~ITMMultiEngine()
{
	while (allData.size()>0)
	{
		delete allData.back();
		allData.pop_back();
	}

	if (renderState_freeview!=NULL) delete renderState_freeview;

	delete denseMapper;
	delete trackingController;

	delete tracker;
	delete imuCalibrator;

	delete lowLevelEngine;
	delete viewBuilder;

	delete view;

	delete visualisationEngine;
}

bool ITMMultiEngine::shouldStartNewArea(void) const
{
	ITMScene<ITMVoxel,ITMVoxelIndex> *scene = allData[primaryDataIdx]->scene;
	int blocksInUse = scene->index.getNumAllocatedVoxelBlocks()-scene->localVBA.lastFreeBlockId - 1;
	if ((activeDataIdx.size() == 1) && (blocksInUse > 5000)) return true;
	if ((activeDataIdx.size() > 1) && (blocksInUse > 10000)) return true;

	return false;
}

void ITMMultiEngine::ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement)
{
	// prepare image and turn it into a depth image
	if (imuMeasurement==NULL) viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter);
	else viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, imuMeasurement);

	if (shouldStartNewArea()) {
		int newIdx = allData.size();
		allData.push_back(new ITMLocalModelData<ITMVoxel,ITMVoxelIndex>(settings, visualisationEngine, trackingController, trackedImageSize));
		if (activeDataIdx.size()<2) activeDataIdx.push_back(newIdx);
		else {
			for (int i = 1; (unsigned)i < activeDataIdx.size(); ++i) activeDataIdx[i-1] = activeDataIdx[i];
			activeDataIdx.back() = newIdx;
		}
		primaryDataIdx = activeDataIdx[0];
	}

	fprintf(stderr, "processing data block:");
	for (int i = 0; (unsigned)i < activeDataIdx.size(); ++i)
	{
		ITMLocalModelData<ITMVoxel,ITMVoxelIndex> *activeData = allData[activeDataIdx[i]];
		int blocksInUse = activeData->scene->index.getNumAllocatedVoxelBlocks()-activeData->scene->localVBA.lastFreeBlockId - 1;
		fprintf(stderr, " %i%s (%i)", activeDataIdx[i], (activeDataIdx[i]==primaryDataIdx)?"*":"", blocksInUse);

		// tracking
		trackingController->Track(activeData->trackingState, view);

		// fusion
		denseMapper->ProcessFrame(view, activeData->trackingState, activeData->scene, activeData->renderState);

		// raycast to renderState_live for tracking and free visualisation
		trackingController->Prepare(activeData->trackingState, activeData->scene, view, activeData->renderState);
	}
fprintf(stderr, "...done!\n");
}

Vector2i ITMMultiEngine::GetImageSize(void) const
{
	return trackedImageSize;
}

void ITMMultiEngine::GetImage(ITMUChar4Image *out, GetImageType getImageType, ITMPose *pose, ITMIntrinsics *intrinsics)
{
	if (view == NULL) return;

	out->Clear();

	switch (getImageType)
	{
	case ITMMultiEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
		out->ChangeDims(view->rgb->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) 
			out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	case ITMMultiEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
		out->ChangeDims(view->depth->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depth->UpdateHostFromDevice();
		ITMVisualisationEngine<ITMVoxel,ITMVoxelIndex>::DepthToUchar4(out, view->depth);
		break;
	case ITMMultiEngine::InfiniTAM_IMAGE_SCENERAYCAST:
	{
		ITMLocalModelData<ITMVoxel,ITMVoxelIndex> *activeData = allData[primaryDataIdx];
		ORUtils::Image<Vector4u> *srcImage = activeData->renderState->raycastImage;
		out->ChangeDims(srcImage->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
			out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);	
		break;
	}
	case ITMMultiEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED:
	case ITMMultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
	case ITMMultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
	{
		IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;
		ITMLocalModelData<ITMVoxel,ITMVoxelIndex> *activeData = allData[primaryDataIdx];
		if (getImageType == ITMMultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME;
		else if (getImageType == ITMMultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
		if (renderState_freeview == NULL) renderState_freeview = visualisationEngine->CreateRenderState(activeData->scene, out->noDims);

		visualisationEngine->FindVisibleBlocks(activeData->scene, pose, intrinsics, renderState_freeview);
		visualisationEngine->CreateExpectedDepths(activeData->scene, pose, intrinsics, renderState_freeview);
		visualisationEngine->RenderImage(activeData->scene, pose, intrinsics, renderState_freeview, renderState_freeview->raycastImage, type);

		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
			out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	}
	};
}

