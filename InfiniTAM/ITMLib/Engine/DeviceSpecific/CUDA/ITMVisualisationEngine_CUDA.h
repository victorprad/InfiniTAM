// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMVisualisationEngine.h"

struct RenderingBlock;

namespace ITMLib
{
	namespace Engine
	{
		template<class TVoxel, class TIndex>
		class ITMVisualisationEngine_CUDA : public ITMVisualisationEngine < TVoxel, TIndex >
		{
		private:
			uint *noTotalPoints_device;

		public:
			ITMVisualisationEngine_CUDA(void);
			~ITMVisualisationEngine_CUDA(void);

			void FindVisibleBlocks(const ITMScene<TVoxel, TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
				ITMRenderState *renderState);
			void CreateExpectedDepths(const ITMScene<TVoxel, TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
				ITMRenderState *renderState);
			void RenderImage(const ITMScene<TVoxel, TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
				const ITMRenderState *renderState, ITMUChar4Image *outputImage, bool useColour);
			void FindSurface(const ITMScene<TVoxel, TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
				const ITMRenderState *renderState);
			void CreatePointCloud(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState,
				ITMRenderState *renderState, bool skipPoints);
			void CreateICPMaps(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState,
				ITMRenderState *renderState);

			ITMRenderState* CreateRenderState(const ITMScene<TVoxel, TIndex> *scene, const Vector2i & imgSize);
		};

		template<class TVoxel>
		class ITMVisualisationEngine_CUDA<TVoxel, ITMVoxelBlockHash> : public ITMVisualisationEngine < TVoxel, ITMVoxelBlockHash >
		{
		private:
			uint *noTotalPoints_device;
			RenderingBlock *renderingBlockList_device;
			uint *noTotalBlocks_device;
			int *noVisibleEntries_device;
		public:
			ITMVisualisationEngine_CUDA(void);
			~ITMVisualisationEngine_CUDA(void);

			void FindVisibleBlocks(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
				ITMRenderState *renderState);
			void CreateExpectedDepths(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
				ITMRenderState *renderState);
			void RenderImage(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
				const ITMRenderState *renderState, ITMUChar4Image *outputImage, bool useColour);
			void FindSurface(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
				const ITMRenderState *renderState);
			void CreatePointCloud(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState,
				ITMRenderState *renderState, bool skipPoints);
			void CreateICPMaps(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState,
				ITMRenderState *renderState);

			ITMRenderState* CreateRenderState(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const Vector2i & imgSize);
		};
	}
}
