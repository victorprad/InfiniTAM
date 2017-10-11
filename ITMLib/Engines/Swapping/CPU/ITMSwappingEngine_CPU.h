// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMSwappingEngine.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMSwappingEngine_CPU : public ITMSwappingEngine < TVoxel, TIndex >
	{
	public:
		void IntegrateGlobalIntoLocal(ITMScene<TVoxel, TIndex> *scene, ITMRenderState *renderState) {}
		void SaveToGlobalMemory(ITMScene<TVoxel, TIndex> *scene, ITMRenderState *renderState) {}
		void CleanLocalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState) {}
	};

	template<class TVoxel>
	class ITMSwappingEngine_CPU<TVoxel, ITMVoxelBlockHash> : public ITMSwappingEngine < TVoxel, ITMVoxelBlockHash >
	{
	private:
		int LoadFromGlobalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene);

	public:
		// This class is currently just for debugging purposes -- swaps CPU memory to CPU memory.
		// Potentially this could stream into the host memory from somwhere else (disk, database, etc.).

		void IntegrateGlobalIntoLocal(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState);
		void SaveToGlobalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState);
		void CleanLocalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState);

		ITMSwappingEngine_CPU(void);
		~ITMSwappingEngine_CPU(void);
	};
}
