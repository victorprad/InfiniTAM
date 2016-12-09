// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMLocalVBA.h"
#include "ITMGlobalCache.h"
#include "../../Utils/ITMSceneParams.h"

namespace ITMLib
{
	/** \brief
	Represents the 3D world model as a hash of small voxel
	blocks
	*/
	template<class TVoxel, class TIndex>
	class ITMScene
	{
	public:
		/** Scene parameters like voxel size etc. */
		const ITMSceneParams *sceneParams;

		/** Hash table to reference the 8x8x8 blocks */
		TIndex index;

		/** Current local content of the 8x8x8 voxel blocks -- stored host or device */
		ITMLocalVBA<TVoxel> localVBA;

		/** Global content of the 8x8x8 voxel blocks -- stored on host only */
		ITMGlobalCache<TVoxel> *globalCache;

		void SaveToDirectory(const std::string &outputDirectory) const
		{
			localVBA.SaveToDirectory(outputDirectory);
			index.SaveToDirectory(outputDirectory);
		}

		void LoadFromDirectory(const std::string &outputDirectory)
		{
			localVBA.LoadFromDirectory(outputDirectory);
			index.LoadFromDirectory(outputDirectory);			
		}

		ITMScene(const ITMSceneParams *_sceneParams, bool _useSwapping, MemoryDeviceType _memoryType)
			: sceneParams(_sceneParams), index(_memoryType), localVBA(_memoryType, index.getNumAllocatedVoxelBlocks(), index.getVoxelBlockSize())
		{
			if (_useSwapping) globalCache = new ITMGlobalCache<TVoxel>();
			else globalCache = NULL;
		}

		~ITMScene(void)
		{
			if (globalCache != NULL) delete globalCache;
		}

		// Suppress the default copy constructor and assignment operator
		ITMScene(const ITMScene&);
		ITMScene& operator=(const ITMScene&);
	};
}
