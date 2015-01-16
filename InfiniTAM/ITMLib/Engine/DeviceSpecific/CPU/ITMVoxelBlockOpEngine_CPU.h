// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMVoxelBlockOpEngine.h"

namespace ITMLib
{
	namespace Engine
	{
		template<class TVoxel, class TIndex>
		class ITMVoxelBlockOpEngine_CPU : public ITMVoxelBlockOpEngine<TVoxel,TIndex>
		{
			void SplitAndMerge(ITMScene<TVoxel,TIndex> *scene) {}
		};

		template<class TVoxel>
		class ITMVoxelBlockOpEngine_CPU<TVoxel,ITMVoxelBlockHHash> : public ITMVoxelBlockOpEngine<TVoxel,ITMVoxelBlockHHash>
		{
		private:
			float *complexities;
			int *blocklist;

			void ComputeComplexities(ITMScene<TVoxel,ITMVoxelBlockHHash> *scene);
			void SplitVoxelBlocks(ITMScene<TVoxel,ITMVoxelBlockHHash> *scene);
			void MergeVoxelBlocks(ITMScene<TVoxel,ITMVoxelBlockHHash> *scene);

		public:
			void SplitAndMerge(ITMScene<TVoxel,ITMVoxelBlockHHash> *scene);

			ITMVoxelBlockOpEngine_CPU(void);
			~ITMVoxelBlockOpEngine_CPU(void);
		};
	}
}
