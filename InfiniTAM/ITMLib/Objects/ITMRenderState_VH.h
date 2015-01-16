// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdlib.h>

#include "ITMRenderState.h"
#include "ITMMemoryBlock.h"

namespace ITMLib
{
	namespace Objects
	{
		/** \brief
		    Stores the render state used by the SceneReconstruction 
			and visualisation engines, as used by voxel hashing.
		*/
		class ITMRenderState_VH : public ITMRenderState
		{
		private:
			MemoryDeviceType memoryType;

			/** A list of "live entries", that are currently
			being processed by integration and tracker.
			*/
			ITMMemoryBlock<int> *liveEntryIDs;

			/** A list of "visible entries", that are
			currently being processed by integration
			and tracker.
			*/
			ITMMemoryBlock<uchar> *entriesVisibleType;

#ifdef COMPILE_WITH_METAL
			void *liveEntryIDs_mb;
			void *entriesVisibleType_mb;
#endif
		public:
			/** Number of entries in the live list. */
			int noLiveEntries;
            
			ITMRenderState_VH(int noTotalEntries, const Vector2i & imgSize, float vf_min, float vf_max, MemoryDeviceType memoryType = MEMORYDEVICE_CPU)
				: ITMRenderState(imgSize, vf_min, vf_max, memoryType)
            {
				this->memoryType = memoryType;

				liveEntryIDs = new ITMMemoryBlock<int>(SDF_LOCAL_BLOCK_NUM, memoryType);
				entriesVisibleType = new ITMMemoryBlock<uchar>(noTotalEntries, memoryType);

				noLiveEntries = 0;
            }
            
			~ITMRenderState_VH()
            {
				delete liveEntryIDs;
				delete entriesVisibleType;
            }

			/** Get the list of "live entries", that are currently
			processed by integration and tracker.
			*/
			const int *GetLiveEntryIDs(void) const { return liveEntryIDs->GetData(memoryType); }
			int *GetLiveEntryIDs(void) { return liveEntryIDs->GetData(memoryType); }

			/** Get the list of "visible entries", that are
			currently processed by integration and tracker.
			*/
			uchar *GetEntriesVisibleType(void) { return entriesVisibleType->GetData(memoryType); }

#ifdef COMPILE_WITH_METAL
			void* GetLiveEntryIDs_MB(void) { return liveEntryIDs_mb; }
			void* GetEntriesVisibleType_MB(void) { return entriesVisibleType_mb; }
#endif
		};
	}
} 