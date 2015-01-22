// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMVoxelBlockOpEngine.h"

namespace ITMLib
{
	namespace Engine
	{
		template<class TVoxel, class TIndex>
		class ITMVoxelBlockOpEngine_CUDA : public ITMVoxelBlockOpEngine<TVoxel,TIndex>
		{
			void SplitAndMerge(ITMScene<TVoxel,TIndex> *scene, const ITMRenderState *renderState) {}
		};

		template<class TVoxel>
		class ITMVoxelBlockOpEngine_CUDA<TVoxel,ITMVoxelBlockHHash> : public ITMVoxelBlockOpEngine<TVoxel,ITMVoxelBlockHHash>
		{
		private:
			float *complexities;
			int *blocklist;
			int *blocklist_size_device;
			int *noAllocatedVoxelEntries_device;
			int *noAllocatedExcessEntries_device;

			void ComputeComplexities(ITMScene<TVoxel,ITMVoxelBlockHHash> *scene, const ITMRenderState *renderState);
			void SplitVoxelBlocks(ITMScene<TVoxel,ITMVoxelBlockHHash> *scene, const ITMRenderState *renderState);
			void MergeVoxelBlocks(ITMScene<TVoxel,ITMVoxelBlockHHash> *scene, const ITMRenderState *renderState);

		public:
			void SplitAndMerge(ITMScene<TVoxel,ITMVoxelBlockHHash> *scene, const ITMRenderState *renderState);

			ITMVoxelBlockOpEngine_CUDA(void);
			~ITMVoxelBlockOpEngine_CUDA(void);
		};
	}
}
