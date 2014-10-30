// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdlib.h>

#include "../Utils/ITMLibDefines.h"

namespace ITMLib
{
	namespace Objects
	{
		/** \brief
			This is the central class for the original fixed size volume
			representation. It contains the data needed on the CPU and
			a pointer to the data structure on the GPU.
			*/
		class ITMPlainVoxelArray
		{
		public:
			struct ITMVoxelArrayInfo {
				/// Size in voxels
				Vector3i size;
				/// offset of the lower left front corner of the volume in voxels
				Vector3i offset;

				ITMVoxelArrayInfo(void)
				{
					size.x = size.y = size.z = 512;
					offset.x = -256;
					offset.y = -256;
					offset.z = 0;
				}
			};

			typedef ITMVoxelArrayInfo IndexData;
			struct IndexCache {};

		private:
			IndexData *indexData_device;
			IndexData indexData_host;

			bool dataIsOnGPU;

		public:
			ITMPlainVoxelArray(bool allocateGPU)
			{
				dataIsOnGPU = allocateGPU;

				if (allocateGPU)
				{
#ifndef COMPILE_WITHOUT_CUDA
					ITMSafeCall(cudaMalloc((void**)&indexData_device, sizeof(IndexData)));
					ITMSafeCall(cudaMemcpy(indexData_device, &indexData_host, sizeof(IndexData), cudaMemcpyHostToDevice));
#endif
				}
				else indexData_device = NULL;
			}

			~ITMPlainVoxelArray(void)
			{
				if (indexData_device != NULL) {
#ifndef COMPILE_WITHOUT_CUDA
					ITMSafeCall(cudaFree(indexData_device));
#endif
				}
			}

			/** Maximum number of total entries. */
			int getNumAllocatedVoxelBlocks(void) { return 1; }
			int getVoxelBlockSize(void) { return indexData_host.size.x * indexData_host.size.y * indexData_host.size.z; }

			const Vector3i getVolumeSize(void) { return indexData_host.size; }

			const IndexData* getIndexData(void) const { if (dataIsOnGPU) return indexData_device; else return &indexData_host; }

			// Suppress the default copy constructor and assignment operator
			ITMPlainVoxelArray(const ITMPlainVoxelArray&);
			ITMPlainVoxelArray& operator=(const ITMPlainVoxelArray&);
		};
	}
}
