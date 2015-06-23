// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMVisualisationEngine.h"

namespace ITMLib
{
	namespace Engine
	{
		template<class TVoxel, class TIndex>
		class ITMVisualisationEngine_CPU : public ITMVisualisationEngine < TVoxel, TIndex >
		{
		public:
			explicit ITMVisualisationEngine_CPU(ITMScene<TVoxel, TIndex> *scene) : ITMVisualisationEngine<TVoxel, TIndex>(scene) { }
			~ITMVisualisationEngine_CPU(void) { }

			void FindVisibleBlocks(const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
			void CreateExpectedDepths(const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
			void RenderImage(const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState, 
				ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE) const;
			void FindSurface(const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const;
			void CreatePointCloud(const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints) const;
			void CreateICPMaps(const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
			void ForwardRender(const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;

			ITMRenderState* CreateRenderState(const Vector2i & imgSize) const;
		};

		template<class TVoxel>
		class ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash> : public ITMVisualisationEngine < TVoxel, ITMVoxelBlockHash >
		{
		public:
			explicit ITMVisualisationEngine_CPU(ITMScene<TVoxel, ITMVoxelBlockHash> *scene) 
				: ITMVisualisationEngine<TVoxel, ITMVoxelBlockHash>(scene) { }
			~ITMVisualisationEngine_CPU(void) { }

			void FindVisibleBlocks(const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
			void CreateExpectedDepths(const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
			void RenderImage(const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState, 
				ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE) const;
			void FindSurface(const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const;
			void CreatePointCloud(const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints) const;
			void CreateICPMaps(const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
			void ForwardRender(const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;

			ITMRenderState_VH* CreateRenderState(const Vector2i & imgSize) const;
		};
	}
}
