// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMBlockProjectionEngine.h"

namespace ITMLib
{
	namespace Engine
	{
		template<class TVoxel, class TIndex>
		class ITMBlockProjectionEngine_CPU : public ITMBlockProjectionEngine<TVoxel,TIndex>
		{
			public:
			void CreateExpectedDepths(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMFloat2Image *minmaxImg);
		};

		template<class TVoxel>
		class ITMBlockProjectionEngine_CPU<TVoxel,ITMVoxelBlockHash> : public ITMBlockProjectionEngine<TVoxel,ITMVoxelBlockHash>
		{
			public:
			void CreateExpectedDepths(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMFloat2Image *minmaxImg);
		};
	}
}
	
