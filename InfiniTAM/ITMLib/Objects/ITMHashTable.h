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
          
#ifndef __METALC__

#ifdef COMPILE_WITH_METAL
            void *entries_all_mb;
            void *excessAllocationList_mb;
#endif
            
            ITMHashTable(bool useCudaAlloc = false)
            {
				this->usedCudaAlloc = useCudaAlloc;

#ifdef COMPILE_WITH_METAL
                allocateMetalData((void**)&entries_all, (void**)&entries_all_mb, noTotalEntries * sizeof(ITMHashEntry), true);
                allocateMetalData((void**)&excessAllocationList, (void**)&excessAllocationList_mb, SDF_EXCESS_LIST_SIZE * sizeof(int), true);
#else
				if (useCudaAlloc)
				{
#ifndef COMPILE_WITHOUT_CUDA
					ITMSafeCall(cudaMalloc((void**)&entries_all, noTotalEntries * sizeof(ITMHashEntry)));
					ITMSafeCall(cudaMalloc((void**)&excessAllocationList, SDF_EXCESS_LIST_SIZE * sizeof(int)));
#endif
				}
				else
				{
					entries_all = new ITMHashEntry[noTotalEntries];
					excessAllocationList = new int[SDF_EXCESS_LIST_SIZE];
				}
#endif
            }
            
            ~ITMHashTable()
            {
#ifdef COMPILE_WITH_METAL
                freeMetalData((void**)&entries_all, (void**)&entriesVisibleType_mb, noTotalEntries * sizeof(ITMHashEntry), true);
                freeMetalData((void**)&excessAllocationList, (void**)&excessAllocationList_mb, SDF_EXCESS_LIST_SIZE * sizeof(int), true);
#else
				if (usedCudaAlloc)
				{
#ifndef COMPILE_WITHOUT_CUDA
					ITMSafeCall(cudaFree(entries_all));
					ITMSafeCall(cudaFree(excessAllocationList));
#endif
				}
				else
				{
					delete entries_all;
					delete excessAllocationList;
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