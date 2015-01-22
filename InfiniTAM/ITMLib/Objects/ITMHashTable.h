// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdlib.h>
#include "../Utils/ITMLibDefines.h"
#include "../../ORUtils/MemoryBlock.h"

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
			public:
			static const CONSTANT(int) noTotalEntries = SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + SDF_EXCESS_LIST_SIZE;
            
			/** The actual data in the hash table. */
			ORUtils::MemoryBlock<ITMHashEntry> *entries;

			/** Identifies which entries of the overflow
			list are allocated. This is used if too
			many hash collisions caused the buckets to
			overflow.
			*/
			ORUtils::MemoryBlock<int> *excessAllocationList;
          
			ITMHashTable(MemoryDeviceType memoryType)
			{
				entries = new ORUtils::MemoryBlock<ITMHashEntry>(noTotalEntries, memoryType);
				excessAllocationList = new ORUtils::MemoryBlock<int>(SDF_EXCESS_LIST_SIZE, memoryType);
			}

			~ITMHashTable()
			{
				delete entries;
				delete excessAllocationList;
			}

			void ResetData(void)
			{
				ITMHashEntry *entries = this->entries->GetData(MEMORYDEVICE_CPU);
				int *excessAllocationList = this->excessAllocationList->GetData(MEMORYDEVICE_CPU);

				memset(entries, 0, noTotalEntries * sizeof(ITMHashEntry));
				for (int i = 0; i < noTotalEntries; i++) { entries[i].ptr = -3; }

				for (int i = 0; i < SDF_EXCESS_LIST_SIZE; i++) excessAllocationList[i] = i;
			}
		};
	}
} 
