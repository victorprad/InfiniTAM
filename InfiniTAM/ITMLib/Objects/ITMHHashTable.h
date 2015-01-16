#pragma once

#include <stdlib.h>

#include "../Utils/ITMLibDefines.h"

#include "ITMImage.h"
#include "ITMPose.h"

namespace ITMLib
{
	namespace Objects
	{
		/** \brief
		    Stores the hash table information, effectively a list of
		    pointers into a ITMLib::Objects::ITMLocalVBA.
		*/
		class ITMHHashTable
		{
			public:
			static const int noTotalEntriesPerLevel = SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + SDF_EXCESS_LIST_SIZE;
			static const int noTotalEntries = (SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + SDF_EXCESS_LIST_SIZE) * SDF_HASH_NO_H_LEVELS;
			static const int noLevels = SDF_HASH_NO_H_LEVELS;

			/** The actual data in the hash table. */
			ITMHashEntry entries_all[noTotalEntries];
			/** Identifies which entries of the overflow
			    list are allocated. This is used if too
			    many hash collisions caused the buckets to
			    overflow.
			*/
			int excessAllocationList[SDF_EXCESS_LIST_SIZE * SDF_HASH_NO_H_LEVELS];
			/** A list of "live entries", that are currently
			    being processed by integration and tracker.
			*/
			int liveEntryIDs[SDF_LOCAL_BLOCK_NUM];
			/** A list of "visible entries", that are
			    currently being processed by integration
			    and tracker.
			*/
			uchar entriesVisibleType[noTotalEntries];

			_CPU_AND_GPU_CODE_ static int GetLevelForEntry(int entryId)
			{
				return (int)(entryId / noTotalEntriesPerLevel);
			}

			void ResetData(void)
			{
				memset(entries_all, 0, noTotalEntries * sizeof(ITMHashEntry));
				for (int i = 0; i < noTotalEntries; i++) { entries_all[i].ptr = -3; }

				for (int listId = 0; listId < SDF_HASH_NO_H_LEVELS; listId++)
				{
					int startPoint = listId * SDF_EXCESS_LIST_SIZE;
					for (int i = 0; i < SDF_EXCESS_LIST_SIZE; i++) excessAllocationList[startPoint + i] = i;
				}
			}
		};
	}
}
