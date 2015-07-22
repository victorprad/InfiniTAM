// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdlib.h>
#include <stdio.h>

#include "../Utils/ITMLibDefines.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "../../ORUtils/CUDADefines.h"
#endif

namespace ITMLib
{
	namespace Objects
	{
		template<class TVoxel>
		class ITMGlobalCache
		{
		private:
			bool *hasStoredData;
			TVoxel *storedVoxelBlocks;
			ITMHashSwapState *swapStates_host, *swapStates_device;

			bool *hasSyncedData_host, *hasSyncedData_device;
			TVoxel *syncedVoxelBlocks_host, *syncedVoxelBlocks_device;

			int *neededEntryIDs_host, *neededEntryIDs_device;
		public:
			inline void SetStoredData(int address, TVoxel *data) 
			{ 
				hasStoredData[address] = true; 
				memcpy(storedVoxelBlocks + address * SDF_BLOCK_SIZE3, data, sizeof(TVoxel) * SDF_BLOCK_SIZE3);
			}
			inline bool HasStoredData(int address) const { return hasStoredData[address]; }
			inline TVoxel *GetStoredVoxelBlock(int address) { return storedVoxelBlocks + address * SDF_BLOCK_SIZE3; }

			bool *GetHasSyncedData(bool useGPU) const { return useGPU ? hasSyncedData_device : hasSyncedData_host; }
			TVoxel *GetSyncedVoxelBlocks(bool useGPU) const { return useGPU ? syncedVoxelBlocks_device : syncedVoxelBlocks_host; }

			ITMHashSwapState *GetSwapStates(bool useGPU) { return useGPU ? swapStates_device : swapStates_host; }
			int *GetNeededEntryIDs(bool useGPU) { return useGPU ? neededEntryIDs_device : neededEntryIDs_host; }

			int noTotalEntries; 

			ITMGlobalCache() : noTotalEntries(SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE)
			{	
				hasStoredData = (bool*)malloc(noTotalEntries * sizeof(bool));
				storedVoxelBlocks = (TVoxel*)malloc(noTotalEntries * sizeof(TVoxel) * SDF_BLOCK_SIZE3);
				memset(hasStoredData, 0, noTotalEntries);

				swapStates_host = (ITMHashSwapState *)malloc(noTotalEntries * sizeof(ITMHashSwapState));
				memset(swapStates_host, 0, sizeof(ITMHashSwapState) * noTotalEntries);

#ifndef COMPILE_WITHOUT_CUDA
				ITMSafeCall(cudaMallocHost((void**)&syncedVoxelBlocks_host, SDF_TRANSFER_BLOCK_NUM * sizeof(TVoxel) * SDF_BLOCK_SIZE3));
				ITMSafeCall(cudaMallocHost((void**)&hasSyncedData_host, SDF_TRANSFER_BLOCK_NUM * sizeof(bool)));
				ITMSafeCall(cudaMallocHost((void**)&neededEntryIDs_host, SDF_TRANSFER_BLOCK_NUM * sizeof(int)));

				ITMSafeCall(cudaMalloc((void**)&swapStates_device, noTotalEntries * sizeof(ITMHashSwapState)));
				ITMSafeCall(cudaMemset(swapStates_device, 0, noTotalEntries * sizeof(ITMHashSwapState)));

				ITMSafeCall(cudaMalloc((void**)&syncedVoxelBlocks_device, SDF_TRANSFER_BLOCK_NUM * sizeof(TVoxel) * SDF_BLOCK_SIZE3));
				ITMSafeCall(cudaMalloc((void**)&hasSyncedData_device, SDF_TRANSFER_BLOCK_NUM * sizeof(bool)));

				ITMSafeCall(cudaMalloc((void**)&neededEntryIDs_device, SDF_TRANSFER_BLOCK_NUM * sizeof(int)));
#else
				syncedVoxelBlocks_host = (TVoxel *)malloc(SDF_TRANSFER_BLOCK_NUM * sizeof(TVoxel) * SDF_BLOCK_SIZE3);
				hasSyncedData_host = (bool*)malloc(SDF_TRANSFER_BLOCK_NUM * sizeof(bool));
				neededEntryIDs_host = (int*)malloc(SDF_TRANSFER_BLOCK_NUM * sizeof(int));
#endif
			}

			void SaveToFile(char *fileName) const
			{
				TVoxel *storedData = storedVoxelBlocks;

				FILE *f = fopen(fileName, "wb");

				fwrite(hasStoredData, sizeof(bool), noTotalEntries, f);
				for (int i = 0; i < noTotalEntries; i++)
				{
					fwrite(storedData, sizeof(TVoxel) * SDF_BLOCK_SIZE3, 1, f);
					storedData += SDF_BLOCK_SIZE3;
				}

				fclose(f);
			}

			void ReadFromFile(char *fileName)
			{
				TVoxel *storedData = storedVoxelBlocks;
				FILE *f = fopen(fileName, "rb");

				size_t tmp = fread(hasStoredData, sizeof(bool), noTotalEntries, f);
				if (tmp == (size_t)noTotalEntries) {
					for (int i = 0; i < noTotalEntries; i++)
					{
						fread(storedData, sizeof(TVoxel) * SDF_BLOCK_SIZE3, 1, f);
						storedData += SDF_BLOCK_SIZE3;
					}
				}

				fclose(f);
			}

			~ITMGlobalCache(void) 
			{
				free(hasStoredData);
				free(storedVoxelBlocks);

				free(swapStates_host);

#ifndef COMPILE_WITHOUT_CUDA
				ITMSafeCall(cudaFreeHost(hasSyncedData_host));
				ITMSafeCall(cudaFreeHost(syncedVoxelBlocks_host));
				ITMSafeCall(cudaFreeHost(neededEntryIDs_host));

				ITMSafeCall(cudaFree(swapStates_device));
				ITMSafeCall(cudaFree(syncedVoxelBlocks_device));
				ITMSafeCall(cudaFree(hasSyncedData_device));
				ITMSafeCall(cudaFree(neededEntryIDs_device));
#else
				free(hasSyncedData_host);
				free(syncedVoxelBlocks_host);
				free(neededEntryIDs_host);
#endif
			}
		};
	}
}
