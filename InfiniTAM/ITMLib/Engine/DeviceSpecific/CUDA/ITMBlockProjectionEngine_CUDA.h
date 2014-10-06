// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMBlockProjectionEngine.h"
#include "../../DeviceAgnostic/ITMBlockProjectionEngine.h"

namespace ITMLib
{
	namespace Engine
	{
		template<class TVoxel, class TIndex>
		class ITMBlockProjectionEngine_CUDA : public ITMBlockProjectionEngine<TVoxel,TIndex>
		{
		public:
			void CreateExpectedDepths(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMFloat2Image *minmaxImg);
		};

		template<class TVoxel>
		class ITMBlockProjectionEngine_CUDA<TVoxel,ITMVoxelBlockHash> : public ITMBlockProjectionEngine<TVoxel,ITMVoxelBlockHash>
		{
			RenderingBlock *renderingBlockList_device;
			uint *noTotalBlocks_device;
		public:
			ITMBlockProjectionEngine_CUDA();
			~ITMBlockProjectionEngine_CUDA();

			void CreateExpectedDepths(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMFloat2Image *minmaxImg);
		};
	}
}
	
