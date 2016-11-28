// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Engines/MultiScene/ITMMapGraphManager.h"
#include "../Scene/ITMMultiSceneAccess.h"
#include "../../Objects/RenderStates/ITMRenderState.h"

namespace ITMLib {

	template<class TVoxel, class TIndex>
	class ITMRenderStateMultiScene : public ITMRenderState 
	{
	private:
		MemoryDeviceType memoryType;

	public:
		typedef typename ITMMultiIndex<TIndex>::IndexData MultiIndexData;
		typedef ITMMultiVoxel<TVoxel> MultiVoxelData;
		typedef ITMVoxelMapGraphManager<TVoxel, TIndex> MultiSceneManager;

#ifndef COMPILE_WITHOUT_CUDA
		MultiIndexData *indexData_device;
		MultiVoxelData *voxelData_device;
#endif
		MultiIndexData indexData_host;
		MultiVoxelData voxelData_host;

		ITMSceneParams sceneParams;

		ITMRenderStateMultiScene(const Vector2i &imgSize, float vf_min, float vf_max, MemoryDeviceType _memoryType)
			: ITMRenderState(imgSize, vf_min, vf_max, _memoryType)
		{
			memoryType = _memoryType;

#ifndef COMPILE_WITHOUT_CUDA
			if (memoryType == MEMORYDEVICE_CUDA) {
				ORcudaSafeCall(cudaMalloc((void**)&indexData_device, sizeof(MultiIndexData)));
				ORcudaSafeCall(cudaMalloc((void**)&voxelData_device, sizeof(MultiVoxelData)));
			}
#endif
		}

		~ITMRenderStateMultiScene(void)
		{
#ifndef COMPILE_WITHOUT_CUDA
			if (memoryType == MEMORYDEVICE_CUDA) {
				ORcudaSafeCall(cudaFree(indexData_device));
				ORcudaSafeCall(cudaFree(voxelData_device));
			}
#endif
		}

		void PrepareLocalMaps(const MultiSceneManager & sceneManager)
		{
			sceneParams = *(sceneManager.getLocalMap(0)->scene->sceneParams);

			int num = (int)sceneManager.numLocalMaps();
			if (num > MAX_NUM_LOCALMAPS) num = MAX_NUM_LOCALMAPS;
			indexData_host.numLocalMaps = num;
			for (int localMapId = 0; localMapId < num; ++localMapId) 
			{
				indexData_host.poses_vs[localMapId] = sceneManager.getEstimatedGlobalPose(localMapId).GetM();
				indexData_host.poses_vs[localMapId].m30 /= sceneParams.voxelSize;
				indexData_host.poses_vs[localMapId].m31 /= sceneParams.voxelSize;
				indexData_host.poses_vs[localMapId].m32 /= sceneParams.voxelSize;
				indexData_host.posesInv[localMapId] = sceneManager.getEstimatedGlobalPose(localMapId).GetInvM();
				indexData_host.index[localMapId] = sceneManager.getLocalMap(localMapId)->scene->index.getIndexData();
				voxelData_host.voxels[localMapId] = sceneManager.getLocalMap(localMapId)->scene->localVBA.GetVoxelBlocks();
			}

#ifndef COMPILE_WITHOUT_CUDA
			if (memoryType == MEMORYDEVICE_CUDA) {
				ORcudaSafeCall(cudaMemcpy(indexData_device, &(indexData_host), sizeof(MultiIndexData), cudaMemcpyHostToDevice));
				ORcudaSafeCall(cudaMemcpy(voxelData_device, &(voxelData_host), sizeof(MultiVoxelData), cudaMemcpyHostToDevice));
			}
#endif
		}
	};

}

