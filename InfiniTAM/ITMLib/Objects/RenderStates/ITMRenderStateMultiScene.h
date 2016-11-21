// Copyright 2016 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Engines/MultiScene/ITMMultiSceneManager.h"
#include "../Scene/ITMMultiSceneAccess.h"
#include "../../Objects/RenderStates/ITMRenderState.h"

namespace ITMLib {

	template<class TVoxel, class TIndex>
	class ITMRenderStateMultiScene : public ITMRenderState {
	private:
		MemoryDeviceType memoryType;

	public:
		typedef typename ITMMultiIndex<TIndex>::IndexData MultiIndexData;
		typedef ITMMultiVoxel<TVoxel> MultiVoxelData;
		typedef ITMMultiSceneManager_instance<TVoxel, TIndex> MultiSceneManager;

		MultiIndexData *indexData_device, indexData_host;
		MultiVoxelData *voxelData_device, voxelData_host;

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

		void PrepareLocalScenes(const MultiSceneManager & sceneManager)
		{
			sceneParams = *(sceneManager.getScene(0)->scene->sceneParams);

			int num = (int)sceneManager.numScenes();
			if (num > MAX_NUM_SCENES) num = MAX_NUM_SCENES;
			indexData_host.numScenes = num;
			for (int sceneId = 0; sceneId < num; ++sceneId) {
				indexData_host.poses_vs[sceneId] = sceneManager.getEstimatedGlobalPose(sceneId).GetM();
				indexData_host.poses_vs[sceneId].m30 /= sceneParams.voxelSize;
				indexData_host.poses_vs[sceneId].m31 /= sceneParams.voxelSize;
				indexData_host.poses_vs[sceneId].m32 /= sceneParams.voxelSize;
				indexData_host.posesInv[sceneId] = sceneManager.getEstimatedGlobalPose(sceneId).GetInvM();
				indexData_host.index[sceneId] = sceneManager.getScene(sceneId)->scene->index.getIndexData();
				voxelData_host.voxels[sceneId] = sceneManager.getScene(sceneId)->scene->localVBA.GetVoxelBlocks();
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

