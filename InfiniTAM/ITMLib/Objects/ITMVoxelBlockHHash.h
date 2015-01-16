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
			typedef ITMHHashTable IndexData;

			struct IndexCache {
				Vector3i blockPos;
				int blockPtr; int blockSize;
				_CPU_AND_GPU_CODE_ IndexCache(void) : blockPos(0x7fffffff), blockPtr(-1), blockSize(1) {}
			};

			/** Maximum number of total entries. */
			static const int noVoxelBlocks = IndexData::noTotalEntries;
			static const int voxelBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

			private:
			IndexData *hashData;
			bool dataIsOnGPU;
			int lastFreeExcessListId[SDF_HASH_NO_H_LEVELS];

			public:
			/** Number of entries in the live list. */
			int noLiveEntries;

			int* GetLastFreeExcessListIds(void) { return lastFreeExcessListId; }
			int GetLastFreeExcessListId(int htIndex) { return lastFreeExcessListId[htIndex]; }
			void SetLastFreeExcessListIds(const int *_lastFreeExcessListIds) { for (int l = 0; l < IndexData::noLevels; ++l) this->lastFreeExcessListId[l] = _lastFreeExcessListIds[l]; }
			void SetLastFreeExcessListId(int lastFreeExcessListId, int htIndex) { this->lastFreeExcessListId[htIndex] = lastFreeExcessListId; }

			ITMVoxelBlockHHash(bool allocateGPU)
			{
				this->dataIsOnGPU = allocateGPU;
				this->noLiveEntries = 0;

				IndexData *hashData_host = new IndexData;
				hashData_host->ResetData();

				if (allocateGPU)
				{
#ifndef COMPILE_WITHOUT_CUDA
					ITMSafeCall(cudaMalloc((void**)&hashData, sizeof(IndexData)));
					ITMSafeCall(cudaMemcpy(hashData, hashData_host, sizeof(IndexData), cudaMemcpyHostToDevice));
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
				if (!dataIsOnGPU)
				{
					delete hashData;
				}
				else
				{
#ifndef COMPILE_WITHOUT_CUDA
					ITMSafeCall(cudaFree(hashData));
#endif
				}
			}

			/** Get the list of actual entries in the hash table. */
			_CPU_AND_GPU_CODE_ const ITMHashEntry *GetEntries(void) const { return hashData->entries_all; }
			_CPU_AND_GPU_CODE_ ITMHashEntry *GetEntries(void) { return hashData->entries_all; }
			/** Get the list that identifies which entries of the
			    overflow list are allocated. This is used if too
			    many hash collisions caused the buckets to overflow.
			*/
			const int *GetExcessAllocationList(void) const { return hashData->excessAllocationList; }
			int *GetExcessAllocationList(void) { return hashData->excessAllocationList; }
			/** Get the list of "live entries", that are currently
			    processed by integration and tracker.
			*/
			const int *GetLiveEntryIDs(void) const { return hashData->liveEntryIDs; }
			int *GetLiveEntryIDs(void) { return hashData->liveEntryIDs; }
			/** Get the list of "visible entries", that are
			    currently processed by integration and tracker.
			*/
			uchar *GetEntriesVisibleType(void) { return hashData->entriesVisibleType; }

			/** Maximum number of total entries. */
			int getNumAllocatedVoxelBlocks(void) { return SDF_LOCAL_BLOCK_NUM; }
			int getVoxelBlockSize(void) { return voxelBlockSize; }

			_CPU_AND_GPU_CODE_ const IndexData* getIndexData(void) const { return hashData; }

			// Suppress the default copy constructor and assignment operator
			ITMVoxelBlockHHash(const ITMVoxelBlockHHash&);
			ITMVoxelBlockHHash& operator=(const ITMVoxelBlockHHash&);
		};
	}
}
