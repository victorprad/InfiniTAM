// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMVisualisationEngine.h"

struct RenderingBlock;

namespace ITMLib
{
	namespace Engine
	{
		template<class TVoxel, class TIndex>
		class ITMVisualisationEngine_CUDA : public ITMVisualisationEngine<TVoxel,TIndex>
		{
		private:
			uint *noTotalPoints_device;

		public:
			ITMVisualisationEngine_CUDA(void);
			~ITMVisualisationEngine_CUDA(void);

			ITMVisualisationState* allocateInternalState(const Vector2i & imgSize)
			{ return new ITMVisualisationState(imgSize, true); }

			void FindVisibleBlocks(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMVisualisationState *state);
			void CreateExpectedDepths(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMFloat2Image *minmaxImg, const ITMVisualisationState *state = NULL);

			void RenderImage(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMVisualisationState *state, ITMUChar4Image *outputImage, bool useColour);
			void CreatePointCloud(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, bool skipPoints);
			void CreateICPMaps(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState);
		};

		template<class TVoxel>
		class ITMVisualisationEngine_CUDA<TVoxel,ITMVoxelBlockHash> : public ITMVisualisationEngine<TVoxel,ITMVoxelBlockHash>
		{
		private:
			uint *noTotalPoints_device;
			RenderingBlock *renderingBlockList_device;
			uint *noTotalBlocks_device;
		public:
			class State : public ITMVisualisationState {
				public:
				State(const Vector2i & imgSize);
				~State(void);

				uchar *entriesVisibleType;
				int *visibleEntryIDs;
				int visibleEntriesNum;
				int *visibleEntriesNum_ptr;
			};

			ITMVisualisationEngine_CUDA(void);
			~ITMVisualisationEngine_CUDA(void);

			ITMVisualisationState* allocateInternalState(const Vector2i & imgSize)
			{ return new State(imgSize); }

			void FindVisibleBlocks(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMVisualisationState *state);
			void CreateExpectedDepths(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMFloat2Image *minmaxImg, const ITMVisualisationState *state = NULL);

			void RenderImage(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMVisualisationState *state, ITMUChar4Image *outputImage, bool useColour);
			void CreatePointCloud(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, bool skipPoints);
			void CreateICPMaps(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState);
		};
	}
}
