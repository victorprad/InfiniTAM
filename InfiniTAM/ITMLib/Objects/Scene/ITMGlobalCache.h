// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdlib.h>
#include <stdio.h>

#include "ITMVoxelBlockHash.h"
#include "../../../ORUtils/CUDADefines.h"

namespace ITMLib
{
	struct ITMHashSwapState
	{
		/// 0 - most recent data is on host, data not currently in active
		///     memory
		/// 1 - data both on host and in active memory, information has not
		///     yet been combined
		/// 2 - most recent data is in active memory, should save this data
		///     back to host at some point
		uchar state;
	};

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
			ORcudaSafeCall(cudaMallocHost((void**)&syncedVoxelBlocks_host, SDF_TRANSFER_BLOCK_NUM * sizeof(TVoxel) * SDF_BLOCK_SIZE3));
			ORcudaSafeCall(cudaMallocHost((void**)&hasSyncedData_host, SDF_TRANSFER_BLOCK_NUM * sizeof(bool)));
			ORcudaSafeCall(cudaMallocHost((void**)&neededEntryIDs_host, SDF_TRANSFER_BLOCK_NUM * sizeof(int)));

			ORcudaSafeCall(cudaMalloc((void**)&swapStates_device, noTotalEntries * sizeof(ITMHashSwapState)));
			ORcudaSafeCall(cudaMemset(swapStates_device, 0, noTotalEntries * sizeof(ITMHashSwapState)));

			ORcudaSafeCall(cudaMalloc((void**)&syncedVoxelBlocks_device, SDF_TRANSFER_BLOCK_NUM * sizeof(TVoxel) * SDF_BLOCK_SIZE3));
			ORcudaSafeCall(cudaMalloc((void**)&hasSyncedData_device, SDF_TRANSFER_BLOCK_NUM * sizeof(bool)));

			ORcudaSafeCall(cudaMalloc((void**)&neededEntryIDs_device, SDF_TRANSFER_BLOCK_NUM * sizeof(int)));
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
			ORcudaSafeCall(cudaFreeHost(hasSyncedData_host));
			ORcudaSafeCall(cudaFreeHost(syncedVoxelBlocks_host));
			ORcudaSafeCall(cudaFreeHost(neededEntryIDs_host));

			ORcudaSafeCall(cudaFree(swapStates_device));
			ORcudaSafeCall(cudaFree(syncedVoxelBlocks_device));
			ORcudaSafeCall(cudaFree(hasSyncedData_device));
			ORcudaSafeCall(cudaFree(neededEntryIDs_device));
#else
			free(hasSyncedData_host);
			free(syncedVoxelBlocks_host);
			free(neededEntryIDs_host);
#endif
		}
	};
}
