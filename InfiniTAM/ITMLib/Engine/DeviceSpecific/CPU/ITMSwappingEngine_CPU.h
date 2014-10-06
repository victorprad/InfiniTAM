// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMSwappingEngine.h"

namespace ITMLib
{
	namespace Engine
	{
		template<class TVoxel, class TIndex>
		class ITMSwappingEngine_CPU : public ITMSwappingEngine<TVoxel,TIndex>
		{
		public:
			void IntegrateGlobalIntoLocal(ITMScene<TVoxel,TIndex> *scene, ITMView *view) {}
			void SaveToGlobalMemory(ITMScene<TVoxel,TIndex> *scene, ITMView *view) {}
		};

		template<class TVoxel>
		class ITMSwappingEngine_CPU<TVoxel,ITMVoxelBlockHash> : public ITMSwappingEngine<TVoxel,ITMVoxelBlockHash>
		{
		protected:
			int DownloadFromGlobalMemory(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, ITMView *view);

		public:
			// This class is currently just for debugging purposes -- swaps CPU memory to CPU memory.
			// Potentially this could stream into the host memory from somwhere else (disk, database, etc.).

			void IntegrateGlobalIntoLocal(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, ITMView *view);
			void SaveToGlobalMemory(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, ITMView *view);

			ITMSwappingEngine_CPU(void);
			~ITMSwappingEngine_CPU(void);
		};
	}
}
