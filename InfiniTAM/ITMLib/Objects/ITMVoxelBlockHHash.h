#pragma once

#include <stdlib.h>

#include "../Utils/ITMLibDefines.h"

#include "ITMHHashTable.h"

namespace ITMLib
{
	namespace Objects
	{
		/** \brief
		    This is the central class for the voxel block hash
		    implementation. It contains all the data needed on the CPU
		    and a pointer to the data structure on the GPU.
		*/
		class ITMVoxelBlockHHash
		{
			public:
			typedef ITMHHashEntry IndexData;

			struct IndexCache {
				Vector3i blockPos;
				int blockPtr; int blockSize;
				_CPU_AND_GPU_CODE_ IndexCache(void) : blockPos(0x7fffffff), blockPtr(-1), blockSize(1) {}
			};

			/** Maximum number of total entries. */
			static const CONSTANT(int) noVoxelBlocks = ITMHHashTable::noTotalEntries;
			static const CONSTANT(int) voxelBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

			private:
			int lastFreeExcessListId[SDF_HASH_NO_H_LEVELS];

			ITMHHashTable *hashData;
			MemoryDeviceType memoryType;

			public:
			int* GetLastFreeExcessListIds(void) { return lastFreeExcessListId; }
			int GetLastFreeExcessListId(int htIndex) { return lastFreeExcessListId[htIndex]; }
			void SetLastFreeExcessListIds(const int *_lastFreeExcessListIds) { for (int l = 0; l < ITMHHashTable::noLevels; ++l) this->lastFreeExcessListId[l] = _lastFreeExcessListIds[l]; }
			void SetLastFreeExcessListId(int lastFreeExcessListId, int htIndex) { this->lastFreeExcessListId[htIndex] = lastFreeExcessListId; }

			ITMVoxelBlockHHash(MemoryDeviceType memoryType)
			{
				this->memoryType = memoryType;

				ITMHHashTable *hashData_host = new ITMHHashTable(MEMORYDEVICE_CPU);
				hashData_host->ResetData();

				if (memoryType == MEMORYDEVICE_CUDA)
				{
#ifndef COMPILE_WITHOUT_CUDA
					hashData = new ITMHHashTable(MEMORYDEVICE_CUDA);
					hashData->entries->SetFrom(hashData_host->entries, ORUtils::MemoryBlock<ITMHHashEntry>::CPU_TO_CUDA);
					hashData->excessAllocationList->SetFrom(hashData_host->excessAllocationList, ORUtils::MemoryBlock<int>::CPU_TO_CUDA);
#endif
					delete hashData_host;
				}
				else
				{
					hashData = hashData_host;
				}

				for (int i = 0; i < SDF_HASH_NO_H_LEVELS; i++) lastFreeExcessListId[i] = SDF_EXCESS_LIST_SIZE - 1;
			}

			~ITMVoxelBlockHHash(void)	
			{
				delete hashData;
			}

			/** Get the list of actual entries in the hash table. */
			_CPU_AND_GPU_CODE_ const ITMHHashEntry *GetEntries(void) const { return hashData->entries->GetData(memoryType); }
			_CPU_AND_GPU_CODE_ ITMHHashEntry *GetEntries(void) { return hashData->entries->GetData(memoryType); }
			/** Get the list that identifies which entries of the
			    overflow list are allocated. This is used if too
			    many hash collisions caused the buckets to overflow.
			*/
			const int *GetExcessAllocationList(void) const { return hashData->excessAllocationList->GetData(memoryType); }
			int *GetExcessAllocationList(void) { return hashData->excessAllocationList->GetData(memoryType); }

			/** Maximum number of total entries. */
			int getNumAllocatedVoxelBlocks(void) { return SDF_LOCAL_BLOCK_NUM; }
			int getVoxelBlockSize(void) { return voxelBlockSize; }

			_CPU_AND_GPU_CODE_ const IndexData* getIndexData(void) const { return hashData->entries->GetData(memoryType); }

			// Suppress the default copy constructor and assignment operator
			ITMVoxelBlockHHash(const ITMVoxelBlockHHash&);
			ITMVoxelBlockHHash& operator=(const ITMVoxelBlockHHash&);
		};
	}
}
