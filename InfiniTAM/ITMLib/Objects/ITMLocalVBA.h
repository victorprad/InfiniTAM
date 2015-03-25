// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdlib.h>

#include "../Utils/ITMLibDefines.h"
#include "../../ORUtils/MemoryBlock.h"

namespace ITMLib
{
	/** \brief
	Stores the actual voxel content that is referred to by a
	ITMLib::ITMHashTable.
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

			ORUtils::MemoryBlock<TVoxel> *voxelBlocks_host;
			ORUtils::MemoryBlock<int> *allocationList_host;

			voxelBlocks_host = new ORUtils::MemoryBlock<TVoxel>(allocatedSize, MEMORYDEVICE_CPU);
			allocationList_host = new ORUtils::MemoryBlock<int>(allocatedSize, MEMORYDEVICE_CPU);

			TVoxel* voxelBlocks_host_ptr = voxelBlocks_host->GetData(MEMORYDEVICE_CPU);
			int* allocationList_host_ptr = allocationList_host->GetData(MEMORYDEVICE_CPU);

			for (int i = 0; i < noBlocks; i++) allocationList_host_ptr[i] = i;
			for (int i = 0; i < allocatedSize; i++) voxelBlocks_host_ptr[i] = TVoxel();

			lastFreeBlockId = noBlocks - 1;

			if (memoryType == MEMORYDEVICE_CUDA)
			{
#ifndef COMPILE_WITHOUT_CUDA
				voxelBlocks = new ORUtils::MemoryBlock<TVoxel>(allocatedSize, MEMORYDEVICE_CUDA);
				allocationList = new ORUtils::MemoryBlock<int>(allocatedSize, MEMORYDEVICE_CUDA);

				voxelBlocks->SetFrom(voxelBlocks_host, ORUtils::MemoryBlock<TVoxel>::CPU_TO_CUDA);
				allocationList->SetFrom(allocationList_host, ORUtils::MemoryBlock<int>::CPU_TO_CUDA);
#endif
				delete voxelBlocks_host;
				delete allocationList_host;
			}
			else
			{
				voxelBlocks = voxelBlocks_host;
				allocationList = allocationList_host;
			}
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
