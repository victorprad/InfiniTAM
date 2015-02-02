#pragma once

#include <stdlib.h>

#include "../Utils/ITMLibDefines.h"

#include "ITMPose.h"

namespace ITMLib
{
	namespace Objects
	{
		/** Dummy wrapper type to make sure, type inference works. */
		class ITMHHashEntry : public ITMHashEntry
		{};

		/** \brief
		    Stores the hash table information, effectively a list of
		    pointers into a ITMLib::Objects::ITMLocalVBA.
		*/
		class ITMHHashTable
		{
			public:
			static const CONSTANT(int) noTotalEntriesPerLevel = SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE;
			static const CONSTANT(int) noTotalEntries = (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE) * SDF_HASH_NO_H_LEVELS;
			static const int noLevels = SDF_HASH_NO_H_LEVELS;

			/** The actual data in the hash table. */
			ORUtils::MemoryBlock<ITMHHashEntry> *entries;
			/** Identifies which entries of the overflow
			    list are allocated. This is used if too
			    many hash collisions caused the buckets to
			    overflow.
			*/
			ORUtils::MemoryBlock<int> *excessAllocationList;

			ITMHHashTable(MemoryDeviceType memoryType)
			{
				entries = new ORUtils::MemoryBlock<ITMHHashEntry>(noTotalEntries, memoryType);
				excessAllocationList = new ORUtils::MemoryBlock<int>(SDF_EXCESS_LIST_SIZE * SDF_HASH_NO_H_LEVELS, memoryType);
			}

			~ITMHHashTable(void)
			{
				delete entries;
				delete excessAllocationList;
			}

			_CPU_AND_GPU_CODE_ static int GetLevelForEntry(int entryId)
			{
				return (int)(entryId / noTotalEntriesPerLevel);
			}

			void ResetData(void)
			{
				ITMHHashEntry *entries = this->entries->GetData(MEMORYDEVICE_CPU);
				int *excessAllocationList = this->excessAllocationList->GetData(MEMORYDEVICE_CPU);

				memset(entries, 0, noTotalEntries * sizeof(ITMHHashEntry));
				for (int i = 0; i < noTotalEntries; i++) { entries[i].ptr = -3; }

				for (int listId = 0; listId < SDF_HASH_NO_H_LEVELS; listId++)
				{
					int startPoint = listId * SDF_EXCESS_LIST_SIZE;
					for (int i = 0; i < SDF_EXCESS_LIST_SIZE; i++) excessAllocationList[startPoint + i] = i;
				}
			}
		};
	}
}
