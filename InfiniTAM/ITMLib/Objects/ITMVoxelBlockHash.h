// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#ifndef __METALC__
#include <stdlib.h>
#endif

#include "../Utils/ITMLibDefines.h"

#include "../../ORUtils/MemoryBlock.h"

namespace ITMLib
{
	namespace Objects
	{
		/** \brief
		This is the central class for the voxel block hash
		implementation. It contains all the data needed on the CPU
		and a pointer to the data structure on the GPU.
		*/
		class ITMVoxelBlockHash
		{
		public:
			typedef ITMHashEntry IndexData;

			struct IndexCache {
				Vector3i blockPos;
				int blockPtr;
				_CPU_AND_GPU_CODE_ IndexCache(void) : blockPos(0x7fffffff), blockPtr(-1) {}
			};

			/** Maximum number of total entries. */
			static const CONSTPTR(int) noTotalEntries = SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE;
			static const CONSTPTR(int) voxelBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

#ifndef __METALC__
		private:
			int lastFreeExcessListId;

			/** The actual data in the hash table. */
			ORUtils::MemoryBlock<ITMHashEntry> *hashEntries;

			/** Identifies which entries of the overflow
			list are allocated. This is used if too
			many hash collisions caused the buckets to
			overflow.
			*/
			ORUtils::MemoryBlock<int> *excessAllocationList;
        
			MemoryDeviceType memoryType;

		public:
			ITMVoxelBlockHash(MemoryDeviceType memoryType)
			{
				this->memoryType = memoryType;
				hashEntries = new ORUtils::MemoryBlock<ITMHashEntry>(noTotalEntries, memoryType);
				excessAllocationList = new ORUtils::MemoryBlock<int>(SDF_EXCESS_LIST_SIZE, memoryType);
			}

			~ITMVoxelBlockHash(void)
			{
				delete hashEntries;
				delete excessAllocationList;
			}

			/** Get the list of actual entries in the hash table. */
			const ITMHashEntry *GetEntries(void) const { return hashEntries->GetData(memoryType); }
			ITMHashEntry *GetEntries(void) { return hashEntries->GetData(memoryType); }

			const IndexData *getIndexData(void) const { return hashEntries->GetData(memoryType); }
			IndexData *getIndexData(void) { return hashEntries->GetData(memoryType); }

			/** Get the list that identifies which entries of the
			overflow list are allocated. This is used if too
			many hash collisions caused the buckets to overflow.
			*/
			const int *GetExcessAllocationList(void) const { return excessAllocationList->GetData(memoryType); }
			int *GetExcessAllocationList(void) { return excessAllocationList->GetData(memoryType); }

			int GetLastFreeExcessListId(void) { return lastFreeExcessListId; }
			void SetLastFreeExcessListId(int lastFreeExcessListId) { this->lastFreeExcessListId = lastFreeExcessListId; }

#ifdef COMPILE_WITH_METAL
			const void* GetEntries_MB(void) { return hashEntries->GetMetalBuffer(); }
			const void* GetExcessAllocationList_MB(void) { return excessAllocationList->GetMetalBuffer(); }
			const void* getIndexData_MB(void) const { return hashEntries->GetMetalBuffer(); }
#endif

			/** Maximum number of total entries. */
			int getNumAllocatedVoxelBlocks(void) { return SDF_LOCAL_BLOCK_NUM; }
			int getVoxelBlockSize(void) { return SDF_BLOCK_SIZE3; }

			// Suppress the default copy constructor and assignment operator
			ITMVoxelBlockHash(const ITMVoxelBlockHash&);
			ITMVoxelBlockHash& operator=(const ITMVoxelBlockHash&);
#endif
		};
	}
}
