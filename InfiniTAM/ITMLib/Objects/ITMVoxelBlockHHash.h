// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdlib.h>

#include "../Utils/ITMLibDefines.h"

namespace ITMLib
{
	namespace Objects
	{
		/** Dummy wrapper type to make sure, type inference works. */
		class ITMHHashEntry : public ITMHashEntry
		{};

		/** \brief
		    This is the central class for the voxel block hash
		    implementation. It contains all the data needed on the CPU
		    and a pointer to the data structure on the GPU.
		*/
		class ITMVoxelBlockHHash
		{
			public:
			static const CONSTPTR(bool) hasColorCoding = true;

			typedef ITMHHashEntry IndexData;

			struct IndexCache {
				Vector3i blockPos;
				int blockPtr; int blockSize;
				_CPU_AND_GPU_CODE_ IndexCache(void) : blockPos(0x7fffffff), blockPtr(-1), blockSize(1) {}
			};

			/** Maximum number of total entries. */
			static const CONSTPTR(int) noTotalEntriesPerLevel = SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE;
			static const CONSTPTR(int) noLevels = SDF_HASH_NO_H_LEVELS;
			static const CONSTPTR(int) noTotalEntries = (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE) * SDF_HASH_NO_H_LEVELS;
			static const CONSTPTR(int) voxelBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

			_CPU_AND_GPU_CODE_ static int GetLevelForEntry(int entryId)
			{
				return (int)(entryId / noTotalEntriesPerLevel);
			}

			private:
			int lastFreeExcessListId[SDF_HASH_NO_H_LEVELS];

			ORUtils::MemoryBlock<ITMHHashEntry> *hashEntries;
			ORUtils::MemoryBlock<int> *excessAllocationList;

			MemoryDeviceType memoryType;

			public:
			ITMVoxelBlockHHash(MemoryDeviceType memoryType)
			{
				this->memoryType = memoryType;

				hashEntries = new ORUtils::MemoryBlock<ITMHHashEntry>(noTotalEntries, memoryType);
				excessAllocationList = new ORUtils::MemoryBlock<int>(SDF_EXCESS_LIST_SIZE * SDF_HASH_NO_H_LEVELS, memoryType);
			}

			~ITMVoxelBlockHHash(void)	
			{
				delete hashEntries;
				delete excessAllocationList;
			}

			/** Get the list of actual entries in the hash table. */
			const ITMHHashEntry *GetEntries(void) const { return hashEntries->GetData(memoryType); }
			ITMHHashEntry *GetEntries(void) { return hashEntries->GetData(memoryType); }

			const IndexData *getIndexData(void) const { return hashEntries->GetData(memoryType); }
			IndexData *getIndexData(void) { return hashEntries->GetData(memoryType); }

			/** Get the list that identifies which entries of the
			    overflow list are allocated. This is used if too
			    many hash collisions caused the buckets to overflow.
			*/
			const int *GetExcessAllocationList(void) const { return excessAllocationList->GetData(memoryType); }
			int *GetExcessAllocationList(void) { return excessAllocationList->GetData(memoryType); }

			int* GetLastFreeExcessListIds(void) { return lastFreeExcessListId; }
			int GetLastFreeExcessListId(int htIndex) { return lastFreeExcessListId[htIndex]; }
			void SetLastFreeExcessListIds(const int *_lastFreeExcessListIds) { for (int l = 0; l < noLevels; ++l) this->lastFreeExcessListId[l] = _lastFreeExcessListIds[l]; }
			void SetLastFreeExcessListId(int lastFreeExcessListId, int htIndex) { this->lastFreeExcessListId[lastFreeExcessListId] = htIndex; }

			/** Maximum number of total entries. */
			int getNumAllocatedVoxelBlocks(void) { return SDF_LOCAL_BLOCK_NUM; }
			int getVoxelBlockSize(void) { return voxelBlockSize; }

			// Suppress the default copy constructor and assignment operator
			ITMVoxelBlockHHash(const ITMVoxelBlockHHash&);
			ITMVoxelBlockHHash& operator=(const ITMVoxelBlockHHash&);
		};
	}
}
