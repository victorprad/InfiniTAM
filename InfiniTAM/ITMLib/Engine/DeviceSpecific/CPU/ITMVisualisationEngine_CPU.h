// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMVisualisationEngine.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMVisualisationEngine_CPU : public ITMVisualisationEngine < TVoxel, TIndex >
	{
	public:
		explicit ITMVisualisationEngine_CPU(void) { }
		~ITMVisualisationEngine_CPU(void) { }

		void FindVisibleBlocks(const ITMSceneBase *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
		void CreateExpectedDepths(const ITMSceneBase *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
		void RenderImage(const ITMSceneBase *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState, 
			ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE) const;
		void FindSurface(const ITMSceneBase *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const;
		void CreatePointCloud(const ITMSceneBase *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints) const;
		void CreateICPMaps(const ITMSceneBase *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
		void ForwardRender(const ITMSceneBase *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;

		ITMRenderState* CreateRenderState(const ITMSceneBase *scene, const Vector2i & imgSize) const;
	};

	template<class TVoxel>
	class ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash> : public ITMVisualisationEngine < TVoxel, ITMVoxelBlockHash >
	{
	public:
		explicit ITMVisualisationEngine_CPU(void) { }
		~ITMVisualisationEngine_CPU(void) { }

		void FindVisibleBlocks(const ITMSceneBase *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
		void CreateExpectedDepths(const ITMSceneBase *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
		void RenderImage(const ITMSceneBase *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState, 
			ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE) const;
		void FindSurface(const ITMSceneBase *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const;
		void CreatePointCloud(const ITMSceneBase *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints) const;
		void CreateICPMaps(const ITMSceneBase *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
		void ForwardRender(const ITMSceneBase *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;

		ITMRenderState_VH* CreateRenderState(const ITMSceneBase *scene, const Vector2i & imgSize) const;
	};
}
