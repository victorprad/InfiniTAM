// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

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
		class ITMHashTable
		{
			public:
			static const int noTotalEntries = SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + SDF_EXCESS_LIST_SIZE;
			/** The actual data in the hash table. */
			ITMHashEntry entries_all[noTotalEntries];
			/** Identifies which entries of the overflow
			    list are allocated. This is used if too
			    many hash collisions caused the buckets to
			    overflow.
			*/
			int excessAllocationList[SDF_EXCESS_LIST_SIZE];
			/** A list of "live entries", that are currently
			    being processed by integration and tracker.
			*/
			int liveEntryIDs[SDF_LOCAL_BLOCK_NUM];
			/** A list of "visible entries", that are
			    currently being processed by integration
			    and tracker.
			*/
			uchar entriesVisibleType[noTotalEntries];

			void ResetData(void)
			{
				memset(entries_all, 0, noTotalEntries * sizeof(ITMHashEntry));
				for (int i = 0; i < noTotalEntries; i++) { entries_all[i].ptr = -2; }

				for (int i = 0; i < SDF_EXCESS_LIST_SIZE; i++) excessAllocationList[i] = i;

			}
		};
	}
}
