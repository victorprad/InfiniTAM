// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdlib.h>
#include <stdio.h>

#include "../Utils/ITMLibDefines.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "../Engine/DeviceSpecific/CUDA/ITMCUDADefines.h"
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
			ITMHashCacheState *cacheStates_host, *cacheStates_device;

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

			ITMHashCacheState *GetCacheStates(bool useGPU) { return useGPU ? cacheStates_device : cacheStates_host; }
			int *GetNeededEntryIDs(bool useGPU) { return useGPU ? neededEntryIDs_device : neededEntryIDs_host; }

			int noTotalEntries; 

			ITMGlobalCache() : noTotalEntries(SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + SDF_EXCESS_LIST_SIZE)
			{	
				hasStoredData = (bool*)malloc(noTotalEntries * sizeof(bool));
				storedVoxelBlocks = (TVoxel*)malloc(noTotalEntries * sizeof(TVoxel) * SDF_BLOCK_SIZE3);
				memset(hasStoredData, 0, noTotalEntries);

				cacheStates_host = (ITMHashCacheState *)malloc(noTotalEntries * sizeof(ITMHashCacheState));
				memset(cacheStates_host, 0, sizeof(ITMHashCacheState) * noTotalEntries);

#ifndef COMPILE_WITHOUT_CUDA
				ITMSafeCall(cudaMallocHost((void**)&syncedVoxelBlocks_host, SDF_TRANSFER_BLOCK_NUM * sizeof(TVoxel) * SDF_BLOCK_SIZE3));
				ITMSafeCall(cudaMallocHost((void**)&hasSyncedData_host, SDF_TRANSFER_BLOCK_NUM * sizeof(bool)));
				ITMSafeCall(cudaMallocHost((void**)&neededEntryIDs_host, SDF_TRANSFER_BLOCK_NUM * sizeof(int)));

				ITMSafeCall(cudaMalloc((void**)&cacheStates_device, noTotalEntries * sizeof(ITMHashCacheState)));
				ITMSafeCall(cudaMemset(cacheStates_device, 0, noTotalEntries * sizeof(ITMHashCacheState)));

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

				fread(hasStoredData, sizeof(bool), noTotalEntries, f);
				for (int i = 0; i < noTotalEntries; i++)
				{
					fread(storedData, sizeof(TVoxel) * SDF_BLOCK_SIZE3, 1, f);
					storedData += SDF_BLOCK_SIZE3;
				}

				fclose(f);
			}

			~ITMGlobalCache(void) 
			{
				free(hasStoredData);
				free(storedVoxelBlocks);

				free(cacheStates_host);

#ifndef COMPILE_WITHOUT_CUDA
				ITMSafeCall(cudaFreeHost(hasSyncedData_host));
				ITMSafeCall(cudaFreeHost(syncedVoxelBlocks_host));
				ITMSafeCall(cudaFreeHost(neededEntryIDs_host));

				ITMSafeCall(cudaFree(cacheStates_device));
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
