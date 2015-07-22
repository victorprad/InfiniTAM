// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSwappingEngine_CUDA.h"
#include "ITMCUDAUtils.h"
#include "../../DeviceAgnostic/ITMSwappingEngine.h"
#include "../../../Objects/ITMRenderState_VH.h"

using namespace ITMLib::Engine;

__global__ void buildListToSwapIn_device(int *neededEntryIDs, int *noNeededEntries, ITMHashSwapState *swapStates, int noTotalEntries);

template<class TVoxel>
__global__ void integrateOldIntoActiveData_device(TVoxel *localVBA, ITMHashSwapState *swapStates, TVoxel *syncedVoxelBlocks_local,
	int *neededEntryIDs_local, ITMHashEntry *hashTable, int maxW);

__global__ void buildListToSwapOut_device(int *neededEntryIDs, int *noNeededEntries, ITMHashSwapState *swapStates,
	ITMHashEntry *hashTable, uchar *entriesVisibleType, int noTotalEntries);

template<class TVoxel>
__global__ void cleanMemory_device(int *voxelAllocationList, int *noAllocatedVoxelEntries, ITMHashSwapState *swapStates,
	ITMHashEntry *hashTable, TVoxel *localVBA, int *neededEntryIDs_local, int noNeededEntries);

template<class TVoxel>
__global__ void moveActiveDataToTransferBuffer_device(TVoxel *syncedVoxelBlocks_local, bool *hasSyncedData_local,
	int *neededEntryIDs_local, ITMHashEntry *hashTable, TVoxel *localVBA);

template<class TVoxel>
ITMSwappingEngine_CUDA<TVoxel,ITMVoxelBlockHash>::ITMSwappingEngine_CUDA(void)
{
	ITMSafeCall(cudaMalloc((void**)&noAllocatedVoxelEntries_device, sizeof(int)));
	ITMSafeCall(cudaMalloc((void**)&noNeededEntries_device, sizeof(int)));
}

template<class TVoxel>
ITMSwappingEngine_CUDA<TVoxel,ITMVoxelBlockHash>::~ITMSwappingEngine_CUDA(void)
{
	ITMSafeCall(cudaFree(noAllocatedVoxelEntries_device));
	ITMSafeCall(cudaFree(noNeededEntries_device));
}

template<class TVoxel>
int ITMSwappingEngine_CUDA<TVoxel,ITMVoxelBlockHash>::LoadFromGlobalMemory(ITMScene<TVoxel,ITMVoxelBlockHash> *scene)
{
	ITMGlobalCache<TVoxel> *globalCache = scene->globalCache;

	ITMHashSwapState *swapStates = globalCache->GetSwapStates(true);

	TVoxel *syncedVoxelBlocks_local = globalCache->GetSyncedVoxelBlocks(true);
	bool *hasSyncedData_local = globalCache->GetHasSyncedData(true);
	int *neededEntryIDs_local = globalCache->GetNeededEntryIDs(true);

	TVoxel *syncedVoxelBlocks_global = globalCache->GetSyncedVoxelBlocks(false);
	bool *hasSyncedData_global = globalCache->GetHasSyncedData(false);
	int *neededEntryIDs_global = globalCache->GetNeededEntryIDs(false);

	dim3 blockSize(256);
	dim3 gridSize((int)ceil((float)scene->index.noTotalEntries / (float)blockSize.x));

	ITMSafeCall(cudaMemset(noNeededEntries_device, 0, sizeof(int)));

	buildListToSwapIn_device << <gridSize, blockSize >> >(neededEntryIDs_local, noNeededEntries_device, swapStates,
		scene->globalCache->noTotalEntries);

	int noNeededEntries;
	ITMSafeCall(cudaMemcpy(&noNeededEntries, noNeededEntries_device, sizeof(int), cudaMemcpyDeviceToHost));

	if (noNeededEntries > 0)
	{
		noNeededEntries = MIN(noNeededEntries, SDF_TRANSFER_BLOCK_NUM);
		ITMSafeCall(cudaMemcpy(neededEntryIDs_global, neededEntryIDs_local, sizeof(int) * noNeededEntries, cudaMemcpyDeviceToHost));

		memset(syncedVoxelBlocks_global, 0, noNeededEntries * SDF_BLOCK_SIZE3 * sizeof(TVoxel));
		memset(hasSyncedData_global, 0, noNeededEntries * sizeof(bool));
		for (int i = 0; i < noNeededEntries; i++)
		{
			int entryId = neededEntryIDs_global[i];

			if (globalCache->HasStoredData(entryId))
			{
				hasSyncedData_global[i] = true;
				memcpy(syncedVoxelBlocks_global + i * SDF_BLOCK_SIZE3, globalCache->GetStoredVoxelBlock(entryId), SDF_BLOCK_SIZE3 * sizeof(TVoxel));
			}
		}

		ITMSafeCall(cudaMemcpy(hasSyncedData_local, hasSyncedData_global, sizeof(bool) * noNeededEntries, cudaMemcpyHostToDevice));
		ITMSafeCall(cudaMemcpy(syncedVoxelBlocks_local, syncedVoxelBlocks_global, sizeof(TVoxel) *SDF_BLOCK_SIZE3 * noNeededEntries, cudaMemcpyHostToDevice));
	}

	return noNeededEntries;
}

template<class TVoxel>
void ITMSwappingEngine_CUDA<TVoxel, ITMVoxelBlockHash>::IntegrateGlobalIntoLocal(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState)
{
	ITMGlobalCache<TVoxel> *globalCache = scene->globalCache;

	ITMHashEntry *hashTable = scene->index.GetEntries();

	ITMHashSwapState *swapStates = globalCache->GetSwapStates(true);

	TVoxel *syncedVoxelBlocks_local = globalCache->GetSyncedVoxelBlocks(true);
	int *neededEntryIDs_local = globalCache->GetNeededEntryIDs(true);

	TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();

	int noNeededEntries = this->LoadFromGlobalMemory(scene);

	int maxW = scene->sceneParams->maxW;

	if (noNeededEntries > 0) {
		dim3 blockSize(SDF_BLOCK_SIZE, SDF_BLOCK_SIZE, SDF_BLOCK_SIZE);
		dim3 gridSize(noNeededEntries);

		integrateOldIntoActiveData_device << <gridSize, blockSize >> >(localVBA, swapStates, syncedVoxelBlocks_local,
			neededEntryIDs_local, hashTable, maxW);
	}
}

template<class TVoxel>
void ITMSwappingEngine_CUDA<TVoxel, ITMVoxelBlockHash>::SaveToGlobalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState)
{
	ITMGlobalCache<TVoxel> *globalCache = scene->globalCache;

	ITMHashSwapState *swapStates = globalCache->GetSwapStates(true);

	ITMHashEntry *hashTable = scene->index.GetEntries();
	uchar *entriesVisibleType = ((ITMRenderState_VH*)renderState)->GetEntriesVisibleType();
	
	TVoxel *syncedVoxelBlocks_local = globalCache->GetSyncedVoxelBlocks(true);
	bool *hasSyncedData_local = globalCache->GetHasSyncedData(true);
	int *neededEntryIDs_local = globalCache->GetNeededEntryIDs(true);

	TVoxel *syncedVoxelBlocks_global = globalCache->GetSyncedVoxelBlocks(false);
	bool *hasSyncedData_global = globalCache->GetHasSyncedData(false);
	int *neededEntryIDs_global = globalCache->GetNeededEntryIDs(false);

	TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	int *voxelAllocationList = scene->localVBA.GetAllocationList();

	int noTotalEntries = globalCache->noTotalEntries;
	
	dim3 blockSize, gridSize;
	int noNeededEntries;

	{
		blockSize = dim3(256);
		gridSize = dim3((int)ceil((float)scene->index.noTotalEntries / (float)blockSize.x));

		ITMSafeCall(cudaMemset(noNeededEntries_device, 0, sizeof(int)));

		buildListToSwapOut_device << <gridSize, blockSize >> >(neededEntryIDs_local, noNeededEntries_device, swapStates,
			hashTable, entriesVisibleType, noTotalEntries);

		ITMSafeCall(cudaMemcpy(&noNeededEntries, noNeededEntries_device, sizeof(int), cudaMemcpyDeviceToHost));
	}

	if (noNeededEntries > 0)
	{
		noNeededEntries = MIN(noNeededEntries, SDF_TRANSFER_BLOCK_NUM);
		{
			blockSize = dim3(SDF_BLOCK_SIZE, SDF_BLOCK_SIZE, SDF_BLOCK_SIZE);
			gridSize = dim3(noNeededEntries);

			moveActiveDataToTransferBuffer_device << <gridSize, blockSize >> >(syncedVoxelBlocks_local, hasSyncedData_local,
				neededEntryIDs_local, hashTable, localVBA);
		}

		{
			blockSize = dim3(256);
			gridSize = dim3((int)ceil((float)noNeededEntries / (float)blockSize.x));

			ITMSafeCall(cudaMemcpy(noAllocatedVoxelEntries_device, &scene->localVBA.lastFreeBlockId, sizeof(int), cudaMemcpyHostToDevice));

			cleanMemory_device << <gridSize, blockSize >> >(voxelAllocationList, noAllocatedVoxelEntries_device, swapStates, hashTable, localVBA,
				neededEntryIDs_local, noNeededEntries);

			ITMSafeCall(cudaMemcpy(&scene->localVBA.lastFreeBlockId, noAllocatedVoxelEntries_device, sizeof(int), cudaMemcpyDeviceToHost));
			scene->localVBA.lastFreeBlockId = MAX(scene->localVBA.lastFreeBlockId, 0);
			scene->localVBA.lastFreeBlockId = MIN(scene->localVBA.lastFreeBlockId, SDF_LOCAL_BLOCK_NUM);
		}

		ITMSafeCall(cudaMemcpy(neededEntryIDs_global, neededEntryIDs_local, sizeof(int) * noNeededEntries, cudaMemcpyDeviceToHost));
		ITMSafeCall(cudaMemcpy(hasSyncedData_global, hasSyncedData_local, sizeof(bool) * noNeededEntries, cudaMemcpyDeviceToHost));
		ITMSafeCall(cudaMemcpy(syncedVoxelBlocks_global, syncedVoxelBlocks_local, sizeof(TVoxel) *SDF_BLOCK_SIZE3 * noNeededEntries, cudaMemcpyDeviceToHost));

		for (int entryId = 0; entryId < noNeededEntries; entryId++)
		{
			if (hasSyncedData_global[entryId])
				globalCache->SetStoredData(neededEntryIDs_global[entryId], syncedVoxelBlocks_global + entryId * SDF_BLOCK_SIZE3);
		}
	}
}

__global__ void buildListToSwapIn_device(int *neededEntryIDs, int *noNeededEntries, ITMHashSwapState *swapStates, int noTotalEntries)
{
	int targetIdx = threadIdx.x + blockIdx.x * blockDim.x;
	if (targetIdx > noTotalEntries - 1) return;

	__shared__ bool shouldPrefix;

	shouldPrefix = false;
	__syncthreads();

	bool isNeededId = (swapStates[targetIdx].state == 1);

	if (isNeededId) shouldPrefix = true;
	__syncthreads();

	if (shouldPrefix)
	{
		int offset = computePrefixSum_device<int>(isNeededId, noNeededEntries, blockDim.x * blockDim.y, threadIdx.x);
		if (offset != -1 && offset < SDF_TRANSFER_BLOCK_NUM) neededEntryIDs[offset] = targetIdx;
	}
}

__global__ void buildListToSwapOut_device(int *neededEntryIDs, int *noNeededEntries, ITMHashSwapState *swapStates,
	ITMHashEntry *hashTable, uchar *entriesVisibleType, int noTotalEntries)
{
	int targetIdx = threadIdx.x + blockIdx.x * blockDim.x;
	if (targetIdx > noTotalEntries - 1) return;

	__shared__ bool shouldPrefix;

	shouldPrefix = false;
	__syncthreads();

	ITMHashSwapState &swapState = swapStates[targetIdx];

	bool isNeededId = ( swapState.state == 2 &&
		hashTable[targetIdx].ptr >= 0 && entriesVisibleType[targetIdx] == 0);

	if (isNeededId) shouldPrefix = true;
	__syncthreads();

	if (shouldPrefix)
	{
		int offset = computePrefixSum_device<int>(isNeededId, noNeededEntries, blockDim.x * blockDim.y, threadIdx.x);
		if (offset != -1 && offset < SDF_TRANSFER_BLOCK_NUM) neededEntryIDs[offset] = targetIdx;
	}
}

template<class TVoxel>
__global__ void cleanMemory_device(int *voxelAllocationList, int *noAllocatedVoxelEntries, ITMHashSwapState *swapStates,
	ITMHashEntry *hashTable, TVoxel *localVBA, int *neededEntryIDs_local, int noNeededEntries)
{
	int locId = threadIdx.x + blockIdx.x * blockDim.x;
	
	if (locId > noNeededEntries - 1) return;

	int entryDestId = neededEntryIDs_local[locId];
	
	swapStates[entryDestId].state = 0;

	int vbaIdx = atomicAdd(&noAllocatedVoxelEntries[0], 1);
	if (vbaIdx < SDF_LOCAL_BLOCK_NUM - 1)
	{
		voxelAllocationList[vbaIdx + 1] = hashTable[entryDestId].ptr;
		hashTable[entryDestId].ptr = -1;
	}
}

template<class TVoxel>
__global__ void moveActiveDataToTransferBuffer_device(TVoxel *syncedVoxelBlocks_local, bool *hasSyncedData_local,
	int *neededEntryIDs_local, ITMHashEntry *hashTable, TVoxel *localVBA)
{
	int entryDestId = neededEntryIDs_local[blockIdx.x];

	ITMHashEntry &hashEntry = hashTable[entryDestId];

	TVoxel *dstVB = syncedVoxelBlocks_local + blockIdx.x * SDF_BLOCK_SIZE3;
	TVoxel *srcVB = localVBA + hashEntry.ptr * SDF_BLOCK_SIZE3;

	int vIdx = threadIdx.x + threadIdx.y * SDF_BLOCK_SIZE + threadIdx.z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
	dstVB[vIdx] = srcVB[vIdx];
	srcVB[vIdx] = TVoxel();

	if (vIdx == 0) hasSyncedData_local[blockIdx.x] = true;
}

template<class TVoxel>
__global__ void integrateOldIntoActiveData_device(TVoxel *localVBA, ITMHashSwapState *swapStates, TVoxel *syncedVoxelBlocks_local,
	int *neededEntryIDs_local, ITMHashEntry *hashTable, int maxW)
{
	int entryDestId = neededEntryIDs_local[blockIdx.x];

	TVoxel *srcVB = syncedVoxelBlocks_local + blockIdx.x * SDF_BLOCK_SIZE3;
	TVoxel *dstVB = localVBA + hashTable[entryDestId].ptr * SDF_BLOCK_SIZE3;

	int vIdx = threadIdx.x + threadIdx.y * SDF_BLOCK_SIZE + threadIdx.z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

	CombineVoxelInformation<TVoxel::hasColorInformation, TVoxel>::compute(srcVB[vIdx], dstVB[vIdx], maxW);

	if (vIdx == 0) swapStates[entryDestId].state = 2;
}

template class ITMLib::Engine::ITMSwappingEngine_CUDA<ITMVoxel, ITMVoxelIndex>;
