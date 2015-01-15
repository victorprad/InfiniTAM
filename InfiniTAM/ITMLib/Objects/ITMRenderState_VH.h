// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdlib.h>

#include "ITMRenderState.h"

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
			bool usedCudaAlloc;

			/** A list of "live entries", that are currently
			being processed by integration and tracker.
			*/
			DEVICEPTR(int) *liveEntryIDs;

			/** A list of "visible entries", that are
			currently being processed by integration
			and tracker.
			*/
			DEVICEPTR(uchar) *entriesVisibleType;

#ifdef COMPILE_WITH_METAL
			void *liveEntryIDs_mb;
			void *entriesVisibleType_mb;
#endif
		public:
			/** Number of entries in the live list. */
			int noLiveEntries;
            
			ITMRenderState_VH(int noTotalEntries, const Vector2i & imgSize, float vf_min, float vf_max, bool useCudaAlloc = false)
				: ITMRenderState(imgSize, vf_min, vf_max, useCudaAlloc)
            {
				this->usedCudaAlloc = useCudaAlloc;

#ifdef COMPILE_WITH_METAL
				allocateMetalData((void**)&liveEntryIDs, (void**)&liveEntryIDs_mb, SDF_LOCAL_BLOCK_NUM * sizeof(int), true);
				allocateMetalData((void**)&entriesVisibleType, (void**)&entriesVisibleType_mb, noTotalEntries * sizeof(uchar), true);
#else
				if (useCudaAlloc)
				{
#ifndef COMPILE_WITHOUT_CUDA
					ITMSafeCall(cudaMalloc((void**)&liveEntryIDs, SDF_LOCAL_BLOCK_NUM * sizeof(int)));
					ITMSafeCall(cudaMalloc((void**)&entriesVisibleType, noTotalEntries * sizeof(uchar)));
#endif
				}
				else
				{
					liveEntryIDs = new int[SDF_LOCAL_BLOCK_NUM];
					entriesVisibleType = new uchar[noTotalEntries];
				}
#endif

				noLiveEntries = 0;
            }
            
			~ITMRenderState_VH()
            {
#ifdef COMPILE_WITH_METAL
				freeMetalData((void**)&liveEntryIDs, (void**)&liveEntryIDs_mb, SDF_LOCAL_BLOCK_NUM * sizeof(int), true);
				freeMetalData((void**)&entriesVisibleType, (void**)&entriesVisibleType_mb, noTotalEntries * sizeof(uchar), true);
#else
				if (usedCudaAlloc)
				{
#ifndef COMPILE_WITHOUT_CUDA
					ITMSafeCall(cudaFree(liveEntryIDs));
					ITMSafeCall(cudaFree(entriesVisibleType));
#endif
				}
				else
				{
					delete [] liveEntryIDs;
					delete [] entriesVisibleType;
				}
#endif
            }

			/** Get the list of "live entries", that are currently
			processed by integration and tracker.
			*/
			const int *GetLiveEntryIDs(void) const { return liveEntryIDs; }
			int *GetLiveEntryIDs(void) { return liveEntryIDs; }

			/** Get the list of "visible entries", that are
			currently processed by integration and tracker.
			*/
			uchar *GetEntriesVisibleType(void) { return entriesVisibleType; }

#ifdef COMPILE_WITH_METAL
			void* GetLiveEntryIDs_MB(void) { return liveEntryIDs_mb; }
			void* GetEntriesVisibleType_MB(void) { return entriesVisibleType_mb; }
#endif
		};
	}
} 