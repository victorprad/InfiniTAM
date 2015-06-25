// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdlib.h>

#include "../Utils/ITMLibDefines.h"
#include "../../ORUtils/MemoryBlock.h"

namespace ITMLib
{
	namespace Objects
	{
		/** \brief
		Stores the actual voxel content that is referred to by a
		ITMLib::Objects::ITMHashTable.
		*/
		template<class TVoxel>
		class ITMLocalVBA
		{
		private:
			ORUtils::MemoryBlock<TVoxel> *voxelBlocks;
			ORUtils::MemoryBlock<int> *allocationList;

			MemoryDeviceType memoryType;

		public:
			inline TVoxel *GetVoxelBlocks(void) { return voxelBlocks->GetData(memoryType); }
			inline const TVoxel *GetVoxelBlocks(void) const { return voxelBlocks->GetData(memoryType); }
			int *GetAllocationList(void) { return allocationList->GetData(memoryType); }

#ifdef COMPILE_WITH_METAL
			const void* GetVoxelBlocks_MB() const { return voxelBlocks->GetMetalBuffer(); }
			const void* GetAllocationList_MB(void) const { return allocationList->GetMetalBuffer(); }
#endif
			int lastFreeBlockId;

			int allocatedSize;

			ITMLocalVBA(MemoryDeviceType memoryType, int noBlocks, int blockSize)
			{
				this->memoryType = memoryType;

				allocatedSize = noBlocks * blockSize;

				voxelBlocks = new ORUtils::MemoryBlock<TVoxel>(allocatedSize, memoryType);
				allocationList = new ORUtils::MemoryBlock<int>(noBlocks, memoryType);
			}

			~ITMLocalVBA(void)
			{
				delete voxelBlocks;
				delete allocationList;
			}

			// Suppress the default copy constructor and assignment operator
			ITMLocalVBA(const ITMLocalVBA&);
			ITMLocalVBA& operator=(const ITMLocalVBA&);
		};
	}
}
