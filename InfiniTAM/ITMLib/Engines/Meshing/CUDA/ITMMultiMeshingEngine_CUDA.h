// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMMultiMeshingEngine.h"
#include "../../../Objects/Scene/ITMMultiSceneAccess.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMMultiMeshingEngine_CUDA : public ITMMultiMeshingEngine<TVoxel, TIndex>
	{
	public:
		void MeshScene(ITMMesh *mesh, const ITMVoxelMapGraphManager<TVoxel, TIndex> & sceneManager) {}
	};

	template<class TVoxel>
	class ITMMultiMeshingEngine_CUDA<TVoxel, ITMVoxelBlockHash> : public ITMMultiMeshingEngine < TVoxel, ITMVoxelBlockHash >
	{
	private:
		unsigned int  *noTriangles_device;
		Vector4s *visibleBlockGlobalPos_device;

	public:
		typedef typename ITMMultiIndex<ITMVoxelBlockHash>::IndexData MultiIndexData;
		typedef ITMMultiVoxel<TVoxel> MultiVoxelData;
		typedef ITMVoxelMapGraphManager<TVoxel, ITMVoxelBlockHash> MultiSceneManager;

		MultiIndexData *indexData_device, indexData_host;
		MultiVoxelData *voxelData_device, voxelData_host;

		void MeshScene(ITMMesh *mesh, const MultiSceneManager & sceneManager);

		ITMMultiMeshingEngine_CUDA(void);
		~ITMMultiMeshingEngine_CUDA(void);
	};
}

