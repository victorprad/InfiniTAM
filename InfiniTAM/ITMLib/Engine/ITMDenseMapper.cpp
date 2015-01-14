// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDenseMapper.h"

#include "../ITMLib.h"

using namespace ITMLib::Engine;

template<class TVoxel, class TIndex>
ITMDenseMapper<TVoxel,TIndex>::ITMDenseMapper(const ITMLibSettings *settings, Vector2i imgSize_rgb, Vector2i imgSize_d)
{
	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

	this->settings = settings;

	this->scene = new ITMScene<ITMVoxel,ITMVoxelIndex>(&(settings->sceneParams), settings->useSwapping, settings->useGPU);

	if (settings->useGPU)
	{
#ifndef COMPILE_WITHOUT_CUDA
		sceneRecoEngine = new ITMSceneReconstructionEngine_CUDA<ITMVoxel,ITMVoxelIndex>();
		if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CUDA<ITMVoxel,ITMVoxelIndex>();
		visualisationEngine = new ITMVisualisationEngine_CUDA<ITMVoxel,ITMVoxelIndex>();
#endif
	}
	else
	{
		sceneRecoEngine = new ITMSceneReconstructionEngine_CPU<ITMVoxel,ITMVoxelIndex>();
		if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CPU<ITMVoxel,ITMVoxelIndex>();
		visualisationEngine = new ITMVisualisationEngine_CPU<ITMVoxel,ITMVoxelIndex>();
	}

	visualisationState = NULL;
}

template<class TVoxel, class TIndex>
ITMDenseMapper<TVoxel,TIndex>::~ITMDenseMapper()
{
	delete sceneRecoEngine;
	delete visualisationEngine;

	if (settings->useSwapping) delete swappingEngine;

	delete scene;
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::ProcessFrame(const ITMView *view, const ITMPose *pose_d)
{
	bool useSwapping = settings->useSwapping;

	// allocation
	sceneRecoEngine->AllocateSceneFromDepth(scene, view, pose_d);

	// integration
	sceneRecoEngine->IntegrateIntoScene(scene, view, pose_d);

	if (useSwapping) {
		// swapping: CPU -> GPU
		swappingEngine->IntegrateGlobalIntoLocal(scene);
		// swapping: GPU -> CPU
		swappingEngine->SaveToGlobalMemory(scene);
	}
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::GetICPMaps(const ITMPose *pose_d, const ITMIntrinsics *intrinsics_d, const ITMView *view, ITMTrackingState *trackingState)
{
	visualisationEngine->CreateExpectedDepths(scene, pose_d, intrinsics_d, trackingState->renderingRangeImage);
	visualisationEngine->CreateICPMaps(scene, view, trackingState);
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::GetPointCloud(const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMView *view, ITMTrackingState *trackingState, bool skipPoints)
{
	visualisationEngine->CreateExpectedDepths(scene, pose, intrinsics, trackingState->renderingRangeImage);
	visualisationEngine->CreatePointCloud(scene, view, trackingState, skipPoints);
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::GetRendering(const ITMPose *pose, const ITMIntrinsics *intrinsics, bool useColour, ITMUChar4Image *out)
{
	if (visualisationState == NULL) visualisationState = visualisationEngine->allocateInternalState(out->noDims);

	visualisationEngine->FindVisibleBlocks(scene, pose, intrinsics, visualisationState);
	visualisationEngine->CreateExpectedDepths(scene, pose, intrinsics, visualisationState->minmaxImage, visualisationState);
	visualisationEngine->RenderImage(scene, pose, intrinsics, visualisationState, visualisationState->outputImage, useColour);

	if (settings->useGPU) visualisationState->outputImage->UpdateHostFromDevice();
	out->SetFrom(visualisationState->outputImage);
}

template class ITMLib::Engine::ITMDenseMapper<ITMVoxel, ITMVoxelIndex>;

