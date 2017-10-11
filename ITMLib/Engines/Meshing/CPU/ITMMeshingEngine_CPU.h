// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMMeshingEngine.h"
#include "../../../Objects/Scene/ITMPlainVoxelArray.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMMeshingEngine_CPU : public ITMMeshingEngine < TVoxel, TIndex >
	{
		void MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, TIndex> *scene) { }
	};

	template<class TVoxel>
	class ITMMeshingEngine_CPU<TVoxel, ITMVoxelBlockHash> : public ITMMeshingEngine < TVoxel, ITMVoxelBlockHash >
	{
	public:
		void MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, ITMVoxelBlockHash> *scene);

		ITMMeshingEngine_CPU(void) { }
		~ITMMeshingEngine_CPU(void) { }
	};
}
