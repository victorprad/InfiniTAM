// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "ITMSceneParams.h"
#include "ITMLocalVBA.h"
#include "ITMGlobalCache.h"

namespace ITMLib
{
	namespace Objects
	{
		class ITMSceneBase {
		public:
			bool useSwapping;

			/** Scene parameters like voxel size etc. */
			const ITMSceneParams *sceneParams;

			ITMSceneBase(const ITMSceneParams *_sceneParams, bool _useSwapping)
			{
				useSwapping = _useSwapping;
				sceneParams = _sceneParams;
			}
			virtual ~ITMSceneBase(void) {}
		};

		/** \brief
		Represents the 3D world model as a hash of small voxel
		blocks
		*/
		template<class TVoxel, class TIndex>
		class ITMScene : public ITMSceneBase
		{
		public:
			/** Hash table to reference the 8x8x8 blocks */
			TIndex index;

			/** Current local content of the 8x8x8 voxel blocks -- stored host or device */
			ITMLocalVBA<TVoxel> localVBA;

			/** Global content of the 8x8x8 voxel blocks -- stored on host only */
			ITMGlobalCache<TVoxel> *globalCache;

			ITMScene(const ITMSceneParams *sceneParams, bool useSwapping, MemoryDeviceType memoryType)
				: ITMSceneBase(sceneParams, useSwapping), index(memoryType), localVBA(memoryType, index.getNumAllocatedVoxelBlocks(), index.getVoxelBlockSize())
			{
				if (useSwapping) globalCache = new ITMGlobalCache<TVoxel>();
			}

			~ITMScene(void)
			{
				if (useSwapping) delete globalCache;
			}

			// Suppress the default copy constructor and assignment operator
			ITMScene(const ITMScene&);
			ITMScene& operator=(const ITMScene&);
		};
	}
}
