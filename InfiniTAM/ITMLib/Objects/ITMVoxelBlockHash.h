// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#ifndef __METALC__
#include <stdlib.h>
#endif

#include "../Utils/ITMLibDefines.h"

#include "ITMHashTable.h"
#include "ITMRenderState.h"

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
			static const CONSTANT(int) noVoxelBlocks = ITMHashTable::noTotalEntries;
			static const CONSTANT(int) voxelBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

		private:
			DEVICEPTR(ITMHashTable) *hashData;
			MemoryDeviceType memoryType;

		public:
			int lastFreeExcessListId;

#ifndef __METALC__
			ITMVoxelBlockHash(MemoryDeviceType memoryType)
			{
				this->memoryType = memoryType;

				ITMHashTable *hashData_host = new ITMHashTable(MEMORYDEVICE_CPU);
				hashData_host->ResetData();

				if (memoryType == MEMORYDEVICE_CUDA)
				{
#ifndef COMPILE_WITHOUT_CUDA
					hashData = new ITMHashTable(MEMORYDEVICE_CUDA);
					hashData->entries->SetFrom(hashData_host->entries, ITMMemoryBlock<ITMHashEntry>::CPU_TO_CUDA);
					hashData->excessAllocationList->SetFrom(hashData_host->excessAllocationList, ITMMemoryBlock<int>::CPU_TO_CUDA);
#endif
					delete hashData_host;
				}
				else hashData = hashData_host;
				lastFreeExcessListId = SDF_EXCESS_LIST_SIZE - 1;
			}

			~ITMVoxelBlockHash(void)
			{
				delete hashData;
			}

			/** Get the list of actual entries in the hash table. */
			const ITMHashEntry *GetEntries(void) const { return hashData->entries->GetData(memoryType); }
			ITMHashEntry *GetEntries(void) { return hashData->entries->GetData(memoryType); }
			
			/** Get the list that identifies which entries of the
			overflow list are allocated. This is used if too
			many hash collisions caused the buckets to overflow.
			*/
			const int *GetExcessAllocationList(void) const { return hashData->excessAllocationList->GetData(memoryType); }
			int *GetExcessAllocationList(void) { return hashData->excessAllocationList->GetData(memoryType); }
			
#ifdef COMPILE_WITH_METAL
			void* GetEntries_MB(void) { return hashData->entries_all_mb; }
			void* GetExcessAllocationList_MB(void) { return hashData->excessAllocationList_mb; }
			void* GetLiveEntryIDs_MB(void) { return hashData->liveEntryIDs_mb; }
			void* GetEntriesVisibleType_MB(void) { return hashData->entriesVisibleType_mb; }
			void* getIndexData_MB(void) const { return hashData->entries_all_mb; }
#endif

			inline const IndexData* getIndexData(void) const { return hashData->entries->GetData(memoryType); }

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