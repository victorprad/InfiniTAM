// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#ifndef __METALC__
#include <stdlib.h>
#endif

#include "../Utils/ITMLibDefines.h"

namespace ITMLib
{
	namespace Objects
	{
		/** \brief
		    Stores the hash table information, effectively a list of
		    pointers into a ITMLib::Objects::ITMLocalVBA.
		*/
		class ITMHashTable
		{
		private:
			bool usedCudaAlloc;

			public:
			static const CONSTANT(int) noTotalEntries = SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + SDF_EXCESS_LIST_SIZE;
            
			/** The actual data in the hash table. */
			DEVICEPTR(ITMHashEntry) *entries_all;
			/** Identifies which entries of the overflow
			    list are allocated. This is used if too
			    many hash collisions caused the buckets to
			    overflow.
			*/
			DEVICEPTR(int) *excessAllocationList;
			/** A list of "live entries", that are currently
			    being processed by integration and tracker.
			*/
			DEVICEPTR(int) *liveEntryIDs;
			/** A list of "visible entries", that are
			    currently being processed by integration
			    and tracker.
			*/
			DEVICEPTR(uchar) *entriesVisibleType;
          
#ifndef __METALC__

#ifdef COMPILE_WITH_METAL
            void *entries_all_mb;
            void *excessAllocationList_mb;
            void *liveEntryIDs_mb;
            void *entriesVisibleType_mb;
#endif
            
            ITMHashTable(bool useCudaAlloc = false)
            {
				this->usedCudaAlloc = useCudaAlloc;

#ifdef COMPILE_WITH_METAL
                allocateMetalData((void**)&entriesVisibleType, (void**)&entriesVisibleType_mb, noTotalEntries * sizeof(uchar), true);
                allocateMetalData((void**)&entries_all, (void**)&entries_all_mb, noTotalEntries * sizeof(ITMHashEntry), true);
                allocateMetalData((void**)&excessAllocationList, (void**)&excessAllocationList_mb, SDF_EXCESS_LIST_SIZE * sizeof(int), true);
                allocateMetalData((void**)&liveEntryIDs, (void**)&liveEntryIDs_mb, SDF_LOCAL_BLOCK_NUM * sizeof(int), true);
#else
				if (useCudaAlloc)
				{
#ifndef COMPILE_WITHOUT_CUDA
					ITMSafeCall(cudaMalloc((void**)&entries_all, noTotalEntries * sizeof(ITMHashEntry)));
					ITMSafeCall(cudaMalloc((void**)&excessAllocationList, SDF_EXCESS_LIST_SIZE * sizeof(int)));
					ITMSafeCall(cudaMalloc((void**)&liveEntryIDs, SDF_LOCAL_BLOCK_NUM * sizeof(int)));
					ITMSafeCall(cudaMalloc((void**)&entriesVisibleType, noTotalEntries * sizeof(uchar)));
#endif
				}
				else
				{
					entries_all = new ITMHashEntry[noTotalEntries];
					excessAllocationList = new int[SDF_EXCESS_LIST_SIZE];
					liveEntryIDs = new int[SDF_LOCAL_BLOCK_NUM];
					entriesVisibleType = new uchar[noTotalEntries];
				}
#endif
            }
            
            ~ITMHashTable()
            {
#ifdef COMPILE_WITH_METAL
                freeMetalData((void**)&entriesVisibleType, (void**)&entriesVisibleType_mb, noTotalEntries * sizeof(uchar), true);
                freeMetalData((void**)&entries_all, (void**)&entriesVisibleType_mb, noTotalEntries * sizeof(ITMHashEntry), true);
                freeMetalData((void**)&excessAllocationList, (void**)&excessAllocationList_mb, SDF_EXCESS_LIST_SIZE * sizeof(int), true);
                freeMetalData((void**)&liveEntryIDs, (void**)&liveEntryIDs_mb, SDF_LOCAL_BLOCK_NUM * sizeof(int), true);
#else
				if (usedCudaAlloc)
				{
#ifndef COMPILE_WITHOUT_CUDA
					ITMSafeCall(cudaFree(entries_all));
					ITMSafeCall(cudaFree(excessAllocationList));
					ITMSafeCall(cudaFree(liveEntryIDs));
					ITMSafeCall(cudaFree(entriesVisibleType));
#endif
				}
				else
				{
					delete entries_all;
					delete excessAllocationList;
					delete liveEntryIDs;
					delete entriesVisibleType;
				}
#endif
            }
            
			void ResetData(void)
			{
				memset(entries_all, 0, noTotalEntries * sizeof(ITMHashEntry));
				for (int i = 0; i < noTotalEntries; i++) { entries_all[i].ptr = -2; }

				for (int i = 0; i < SDF_EXCESS_LIST_SIZE; i++) excessAllocationList[i] = i;
			}
#endif
		};
	}
} 