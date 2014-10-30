// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdlib.h>

#include "../Utils/ITMLibDefines.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "../Engine/DeviceSpecific/CUDA/ITMCUDADefines.h"
#endif

namespace ITMLib
{
	namespace Objects
	{
		/** \brief
		    Stores the actual voxel content that is referred to by a
		    ITMLib::Objects::ITMHashTable.
		*/
		template<class TVoxel>
		class ITMLocalVBA
		{
		private:
			TVoxel *voxelBlocks;
			int *allocationList;
			
			bool dataIsOnGPU;
		public:
			_CPU_AND_GPU_CODE_ inline TVoxel *GetVoxelBlocks(void) { return voxelBlocks; }
			_CPU_AND_GPU_CODE_ inline const TVoxel *GetVoxelBlocks(void) const { return voxelBlocks; }
			int *GetAllocationList(void) { return allocationList; }

			int lastFreeBlockId;

			int allocatedSize;

			ITMLocalVBA(bool allocateGPU, int noBlocks, int blockSize)
			{	
				this->dataIsOnGPU = allocateGPU;

				allocatedSize = noBlocks * blockSize;

				TVoxel *voxelBlocks_host = (TVoxel*)malloc(allocatedSize * sizeof(TVoxel));

				int *allocationList_host = (int*)malloc(allocatedSize * sizeof(int));

				for (int i = 0; i < noBlocks; i++) allocationList_host[i] = i;

				for (int i = 0; i < allocatedSize; i++)
					voxelBlocks_host[i] = TVoxel();

				lastFreeBlockId = noBlocks - 1;

				if (allocateGPU)
				{
#ifndef COMPILE_WITHOUT_CUDA
					ITMSafeCall(cudaMalloc((void**)&voxelBlocks, allocatedSize * sizeof(TVoxel)));
					ITMSafeCall(cudaMalloc((void**)&allocationList, allocatedSize * sizeof(int)));
					ITMSafeCall(cudaMemcpy(allocationList, allocationList_host, allocatedSize * sizeof(int), cudaMemcpyHostToDevice));
					ITMSafeCall(cudaMemcpy(voxelBlocks, voxelBlocks_host, allocatedSize * sizeof(TVoxel), cudaMemcpyHostToDevice));
#endif
					free(voxelBlocks_host);
					free(allocationList_host);
				}
				else
				{
					voxelBlocks = voxelBlocks_host;
					allocationList = allocationList_host;
				}
			}

			~ITMLocalVBA(void) 
			{
				if (!dataIsOnGPU)
				{
					free(voxelBlocks);
					free(allocationList);
				}
				else
				{
#ifndef COMPILE_WITHOUT_CUDA
					ITMSafeCall(cudaFree(voxelBlocks));
					ITMSafeCall(cudaFree(allocationList));
#endif
				}
			}

			// Suppress the default copy constructor and assignment operator
			ITMLocalVBA(const ITMLocalVBA&);
			ITMLocalVBA& operator=(const ITMLocalVBA&);
		};
	}
}
