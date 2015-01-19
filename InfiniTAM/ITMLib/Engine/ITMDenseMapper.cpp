// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDenseMapper.h"
#include "ITMTrackerFactory.h"

#include "../ITMLib.h"

using namespace ITMLib::Engine;

template<class TVoxel, class TIndex>
ITMDenseMapper<TVoxel,TIndex>::ITMDenseMapper(const ITMLibSettings *settings, Vector2i imgSize_rgb, Vector2i imgSize_d)
{
	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

	this->settings = settings;

	MemoryDeviceType memoryType = settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;

	this->scene = new ITMScene<ITMVoxel, ITMVoxelIndex>(&(settings->sceneParams), settings->useSwapping, memoryType);

	switch (settings->deviceType)
	{
	case ITMLibSettings::DEVICE_CPU:
		sceneRecoEngine = new ITMSceneReconstructionEngine_CPU<TVoxel,TIndex>();
		if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CPU<TVoxel,TIndex>();
		visualisationEngine = new ITMVisualisationEngine_CPU<TVoxel,TIndex>();
		break;
	case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
		sceneRecoEngine = new ITMSceneReconstructionEngine_CUDA<TVoxel,TIndex>();
		if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CUDA<TVoxel,TIndex>();
		visualisationEngine = new ITMVisualisationEngine_CUDA<TVoxel,TIndex>();
#endif
		break;
	case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
		sceneRecoEngine = new ITMSceneReconstructionEngine_Metal<TVoxel, TIndex>();
		if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CPU<TVoxel, TIndex>();
		visualisationEngine = new ITMVisualisationEngine_Metal<TVoxel, TIndex>();
#endif
		break;
	}

	this->renderState_live = visualisationEngine->CreateRenderState(scene, ITMTrackerFactory::GetTrackedImageSize(*settings, imgSize_rgb, imgSize_d));
	this->renderState_freeview = NULL;
}

template<class TVoxel, class TIndex>
ITMDenseMapper<TVoxel,TIndex>::~ITMDenseMapper()
{
	delete sceneRecoEngine;
	delete visualisationEngine;

	if (settings->useSwapping) delete swappingEngine;

	delete scene;

	delete renderState_live;
	if (renderState_freeview != NULL) delete renderState_freeview;
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::ProcessFrame(const ITMView *view, const ITMTrackingState *trackingState)
{
	bool useSwapping = settings->useSwapping;

	// allocation
	sceneRecoEngine->AllocateSceneFromDepth(scene, view, trackingState, renderState_live);

	// integration
	sceneRecoEngine->IntegrateIntoScene(scene, view, trackingState, renderState_live);

	if (useSwapping) {
		// swapping: CPU -> GPU
		swappingEngine->IntegrateGlobalIntoLocal(scene, renderState_live);
		// swapping: GPU -> CPU
		swappingEngine->SaveToGlobalMemory(scene, renderState_live);
	}
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::GetICPMaps(const ITMPose *pose_d, const ITMIntrinsics *intrinsics_d, const ITMView *view, ITMTrackingState *trackingState)
{
	visualisationEngine->CreateExpectedDepths(scene, pose_d, intrinsics_d, renderState_live);
	visualisationEngine->CreateICPMaps(scene, view, trackingState, renderState_live);
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::GetPointCloud(const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMView *view, ITMTrackingState *trackingState, bool skipPoints)
{
	visualisationEngine->CreateExpectedDepths(scene, pose, intrinsics, renderState_live);
	visualisationEngine->CreatePointCloud(scene, view, trackingState, renderState_live, skipPoints);
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::GetRendering(const ITMPose *pose, const ITMIntrinsics *intrinsics, bool useColour, ITMUChar4Image *out)
{
	if (renderState_freeview == NULL) renderState_freeview = visualisationEngine->CreateRenderState(scene, out->noDims);

	visualisationEngine->FindVisibleBlocks(scene, pose, intrinsics, renderState_freeview);
	visualisationEngine->CreateExpectedDepths(scene, pose, intrinsics, renderState_freeview);
	visualisationEngine->RenderImage(scene, pose, intrinsics, renderState_freeview, renderState_freeview->raycastImage, useColour);

	if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) renderState_freeview->raycastImage->UpdateHostFromDevice();
	out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
}

template class ITMLib::Engine::ITMDenseMapper<ITMVoxel, ITMVoxelIndex>;

