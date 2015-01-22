// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

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
			void FindVisibleBlocks(const ITMScene<TVoxel, TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, 
				ITMRenderState *renderState);
			void CreateExpectedDepths(const ITMScene<TVoxel, TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, 
				ITMRenderState *renderState);
			void RenderImage(const ITMScene<TVoxel, TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, 
				const ITMRenderState *renderState, ITMUChar4Image *outputImage, bool useColour);
			void CreatePointCloud(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, 
				ITMRenderState *renderState, bool skipPoints);
			void CreateICPMaps(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState,
				ITMRenderState *renderState);

			ITMRenderState* CreateRenderState(const ITMScene<TVoxel, TIndex> *scene, const Vector2i & imgSize);
		};

		template<class TVoxel>
		class ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash> : public ITMVisualisationEngine < TVoxel, ITMVoxelBlockHash >
		{
		public:
			void FindVisibleBlocks(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, 
				ITMRenderState *renderState);
			void CreateExpectedDepths(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, 
				ITMRenderState *renderState);
			void RenderImage(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, 
				const ITMRenderState *renderState, ITMUChar4Image *outputImage, bool useColour);
			void CreatePointCloud(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, 
				ITMRenderState *renderState, bool skipPoints);
			void CreateICPMaps(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState,
				ITMRenderState *renderState);

			ITMRenderState* CreateRenderState(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const Vector2i & imgSize);
		};

		template<class TVoxel>
		class ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHHash> : public ITMVisualisationEngine<TVoxel,ITMVoxelBlockHHash>
		{
		public:
			void FindVisibleBlocks(const ITMScene<TVoxel,ITMVoxelBlockHHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState);
			void CreateExpectedDepths(const ITMScene<TVoxel,ITMVoxelBlockHHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState);
			void RenderImage(const ITMScene<TVoxel,ITMVoxelBlockHHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState, ITMUChar4Image *outputImage, bool useColour);
			void CreatePointCloud(const ITMScene<TVoxel,ITMVoxelBlockHHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints);
			void CreateICPMaps(const ITMScene<TVoxel,ITMVoxelBlockHHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState);

			ITMRenderState* CreateRenderState(const ITMScene<TVoxel, ITMVoxelBlockHHash> *scene, const Vector2i & imgSize);
		};
	}
}
