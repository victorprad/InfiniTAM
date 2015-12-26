// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMMeshingEngine.h"
#include "../../Scene/ITMPlainVoxelArray.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMMeshingEngine_CPU : public ITMMeshingEngine < TVoxel, TIndex >
	{};

	template<class TVoxel>
	class ITMMeshingEngine_CPU<TVoxel, ITMVoxelBlockHash> : public ITMMeshingEngine < TVoxel, ITMVoxelBlockHash >
	{
	public:
		void MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, ITMVoxelBlockHash> *scene);

		ITMMeshingEngine_CPU(void);
		~ITMMeshingEngine_CPU(void);
	};

	template<class TVoxel>
	class ITMMeshingEngine_CPU<TVoxel, ITMPlainVoxelArray> : public ITMMeshingEngine < TVoxel, ITMPlainVoxelArray >
	{
	public:
		void MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, ITMPlainVoxelArray> *scene);

		ITMMeshingEngine_CPU(void);
		~ITMMeshingEngine_CPU(void);
	};
}
