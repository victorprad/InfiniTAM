// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDenseMapper.h"

#include "../Objects/ITMRenderState_VH.h"

#include "../ITMLib.h"

using namespace ITMLib::Engine;

template<class TVoxel, class TIndex>
ITMDenseMapper<TVoxel, TIndex>::ITMDenseMapper(const ITMLibSettings *settings, ITMScene<TVoxel, TIndex>* scene, ITMRenderState *renderState_live)
{
	this->settings = settings;

	this->scene = scene;
	this->renderState_live = renderState_live;

	switch (settings->deviceType)
	{
	case ITMLibSettings::DEVICE_CPU:
		sceneRecoEngine = new ITMSceneReconstructionEngine_CPU<TVoxel,TIndex>();
		if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CPU<TVoxel,TIndex>();
		voxelBlockOpEngine = new ITMVoxelBlockOpEngine_CPU<TVoxel,TIndex>();
		break;
	case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
		sceneRecoEngine = new ITMSceneReconstructionEngine_CUDA<TVoxel,TIndex>();
		if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CUDA<TVoxel,TIndex>();
		voxelBlockOpEngine = new ITMVoxelBlockOpEngine_CUDA<TVoxel,TIndex>();
#endif
		break;
	case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
		sceneRecoEngine = new ITMSceneReconstructionEngine_Metal<TVoxel, TIndex>();
		if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CPU<TVoxel, TIndex>();
		voxelBlockOpEngine = new ITMVoxelBlockOpEngine_CPU<TVoxel,TIndex>();
#endif
		break;
	}
}

template<class TVoxel, class TIndex>
ITMDenseMapper<TVoxel,TIndex>::~ITMDenseMapper()
{
	delete sceneRecoEngine;
	delete voxelBlockOpEngine;

	if (settings->useSwapping) delete swappingEngine;
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::ProcessFrame(const ITMView *view, const ITMTrackingState *trackingState)
{
	bool useSwapping = settings->useSwapping;

	// split and merge voxel blocks according to their complexity
	voxelBlockOpEngine->SplitAndMerge(scene, renderState_live);

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

template class ITMLib::Engine::ITMDenseMapper<ITMVoxel, ITMVoxelIndex>;

