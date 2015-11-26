// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMVoxelBlockOpEngine_CUDA.h"
#include "ITMCUDAUtils.h"
#include "../../DeviceAgnostic/ITMVoxelBlockOpEngine.h"
#include "../../../Objects/ITMRenderState_VH.h"

using namespace ITMLib::Engine;

template<class TVoxel>
ITMVoxelBlockOpEngine_CUDA<TVoxel,ITMVoxelBlockHHash>::ITMVoxelBlockOpEngine_CUDA(void)
{
	ITMSafeCall(cudaMalloc((void**)&complexities, sizeof(float) * ITMVoxelBlockHHash::noTotalEntries));
	ITMSafeCall(cudaMalloc((void**)&blocklist, sizeof(int) * 8 * SDF_LOCAL_BLOCK_NUM));
	ITMSafeCall(cudaMalloc((void**)&blocklist_size_device, sizeof(int)));
	ITMSafeCall(cudaMalloc((void**)&noAllocatedVoxelEntries_device, sizeof(int)));
	ITMSafeCall(cudaMalloc((void**)&noAllocatedExcessEntries_device, sizeof(int)*ITMVoxelBlockHHash::noLevels));

	{
		dim3 blockSize(256);
		dim3 gridSize((int)ceil((float)SDF_LOCAL_BLOCK_NUM / (float)blockSize.x));
		float init = -1.0f;
		memsetKernel_device<float> << <gridSize, blockSize >> >(complexities, init, ITMVoxelBlockHHash::noTotalEntries);
	}
}

template<class TVoxel>
ITMVoxelBlockOpEngine_CUDA<TVoxel,ITMVoxelBlockHHash>::~ITMVoxelBlockOpEngine_CUDA(void) 
{
	ITMSafeCall(cudaFree(noAllocatedExcessEntries_device));
	ITMSafeCall(cudaFree(noAllocatedVoxelEntries_device));
	ITMSafeCall(cudaFree(blocklist_size_device));
	ITMSafeCall(cudaFree(blocklist));
	ITMSafeCall(cudaFree(complexities));
}

template<class TVoxel>
__global__ void computeComplexity_device(const int *liveEntryIDs, const ITMHashEntry *hashEntries, const TVoxel *voxelBlocks, float *complexities)
{
	int htIdx = liveEntryIDs[blockIdx.x];
	int blockId = hashEntries[htIdx].ptr;

	const TVoxel *voxelBlock = &voxelBlocks[blockId * SDF_BLOCK_SIZE3];

	__shared__ float dim_shared[3*SDF_BLOCK_SIZE3];
	Vector3f X_sum;
	float XXT_triangle_sum[3+2+1];

	Vector3f X; float XXT_triangle[3+2+1];
	Vector3i loc(threadIdx.x, threadIdx.y, threadIdx.z);
	ComputePerVoxelSumAndCovariance(loc, voxelBlock, X, XXT_triangle);

	int locId_local = loc.x + loc.y * blockDim.x + loc.z * blockDim.x * blockDim.y;

	{
		dim_shared[locId_local*3+0] = X.v[0];
		dim_shared[locId_local*3+1] = X.v[1];
		dim_shared[locId_local*3+2] = X.v[2];
		__syncthreads();

		int sdataTargetOffset;
		for (uint s = SDF_BLOCK_SIZE3 >> 1; s > 32; s >>= 1)
		{
			sdataTargetOffset = locId_local + s;
			if ((locId_local < s) && (sdataTargetOffset < blockDim.x*blockDim.y*blockDim.z))
			{
				dim_shared[locId_local*3+0] += dim_shared[sdataTargetOffset*3+0];
				dim_shared[locId_local*3+1] += dim_shared[sdataTargetOffset*3+1];
				dim_shared[locId_local*3+2] += dim_shared[sdataTargetOffset*3+2];
			}
			__syncthreads();
		}
		if (locId_local < 32) warpReduce3(dim_shared, locId_local);

		if (locId_local == 0) { 
			X_sum.v[0] = dim_shared[0];
			X_sum.v[1] = dim_shared[1];
			X_sum.v[2] = dim_shared[2];
		}
		__syncthreads();
	}

	for (int paraId = 0; paraId < 6; paraId+=3) {
		dim_shared[locId_local*3+0] = XXT_triangle[paraId+0];
		dim_shared[locId_local*3+1] = XXT_triangle[paraId+1];
		dim_shared[locId_local*3+2] = XXT_triangle[paraId+2];
		__syncthreads();

		int sdataTargetOffset;
		for (uint s = SDF_BLOCK_SIZE3 >> 1; s > 32; s >>= 1)
		{
			sdataTargetOffset = locId_local + s;
			if ((locId_local < s) && (sdataTargetOffset < blockDim.x*blockDim.y*blockDim.z))
			{
				dim_shared[locId_local*3+0] += dim_shared[sdataTargetOffset*3+0];
				dim_shared[locId_local*3+1] += dim_shared[sdataTargetOffset*3+1];
				dim_shared[locId_local*3+2] += dim_shared[sdataTargetOffset*3+2];
			}
			__syncthreads();
		}
		if (locId_local < 32) warpReduce3(dim_shared, locId_local);

		if (locId_local == 0) {
			XXT_triangle_sum[paraId+0] = dim_shared[0];
			XXT_triangle_sum[paraId+1] = dim_shared[1];
			XXT_triangle_sum[paraId+2] = dim_shared[2];
		}
		__syncthreads();
	}

	if (locId_local == 0) complexities[htIdx] = ComputeCovarianceDet(X_sum, XXT_triangle_sum);
}

template<class TVoxel>
void ITMVoxelBlockOpEngine_CUDA<TVoxel,ITMVoxelBlockHHash>::ComputeComplexities(ITMScene<TVoxel,ITMVoxelBlockHHash> *scene, const ITMRenderState *renderState)
{
	const ITMRenderState_VH *renderState_vh = (const ITMRenderState_VH*)renderState;

	TVoxel *voxelBlocks = scene->localVBA.GetVoxelBlocks();
	ITMHHashEntry *hashEntries = scene->index.GetEntries();

	const int *liveList = renderState_vh->GetVisibleEntryIDs();
	int liveListSize = renderState_vh->noVisibleEntries;

	dim3 blockSize(SDF_BLOCK_SIZE-1, SDF_BLOCK_SIZE-1, SDF_BLOCK_SIZE-1);
	dim3 gridSize(liveListSize);

	if (liveListSize > 0) {
		computeComplexity_device << <gridSize, blockSize >> >(liveList, hashEntries, voxelBlocks, complexities);
	}
}

__global__ void createSplits_device(const int *liveList, int liveListSize, float *complexities, ITMHHashEntry *allHashEntries, int *excessAllocationList, int *lastFreeExcessListIds, int *voxelAllocationList, int *lastFreeVoxelBlockId, int *blocklist, int *lastEntryBlockList)
{
	int listIdx = blockIdx.x * blockDim.x + threadIdx.x;
	if (listIdx >= liveListSize) return;

	int htIdx = liveList[listIdx];
	int parentLevel = ITMVoxelBlockHHash::GetLevelForEntry(htIdx);

	// finest level doesn't need splitting...
	if (parentLevel == 0) return;

	if (complexities[htIdx] <= threshold_split) return;
	complexities[htIdx] = -1;

	int childLevel = parentLevel-1;

	ITMHHashEntry *childHashTable = &(allHashEntries[ITMVoxelBlockHHash::noTotalEntriesPerLevel * childLevel]);
	int *childExcessAllocationList = excessAllocationList + (childLevel * SDF_EXCESS_LIST_SIZE);

	createSplitOperations(allHashEntries, childHashTable, childExcessAllocationList, lastFreeExcessListIds + childLevel, voxelAllocationList, lastFreeVoxelBlockId, blocklist, lastEntryBlockList, htIdx, parentLevel);
}

template<class TVoxel>
__global__ void performSplitOperations_device(const int *blocklist, TVoxel *voxelBlocks)
{
	extern __shared__ TVoxel localCopy[];

	int locId = threadIdx.x + threadIdx.y*SDF_BLOCK_SIZE + threadIdx.z*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;
	int blockListPos = blockIdx.x * 8;
	int parentBlockId = blocklist[blockListPos];
	TVoxel *voxelBlock_parent = &(voxelBlocks[parentBlockId * (SDF_BLOCK_SIZE3)]);

	// create backup
	localCopy[locId] = voxelBlock_parent[locId];
	__syncthreads();

	// interpolation of SDF value, but nothing else
	// thread safe and seems to work reasonably well
	for (int child = 0; child < 8; ++child) {
		TVoxel *voxelBlock_child = &(voxelBlocks[blocklist[blockListPos++] * (SDF_BLOCK_SIZE3)]);

		Vector3i loc_parent(threadIdx.x/2 + ((child&1)?4:0), threadIdx.y/2 + ((child&2)?4:0), threadIdx.z/2 + ((child&4)?4:0));
		Vector3u doInterpolate(threadIdx.x&1, threadIdx.y&1, threadIdx.z&1);
		if (loc_parent.x == 7) doInterpolate.x = 0;
		if (loc_parent.y == 7) doInterpolate.y = 0;
		if (loc_parent.z == 7) doInterpolate.z = 0;

		int locId_parent = loc_parent.x + loc_parent.y * SDF_BLOCK_SIZE + loc_parent.z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

		TVoxel res = localCopy[locId_parent];
		float sdf = TVoxel::SDF_valueToFloat(res.sdf)*(float)res.w_depth;
		int w_sum = res.w_depth;

		if (doInterpolate.x) {
			sdf += TVoxel::SDF_valueToFloat(localCopy[locId_parent + 1].sdf)*(float)localCopy[locId_parent + 1].w_depth;
			w_sum += localCopy[locId_parent + 1].w_depth;
		}
		if (doInterpolate.y) {
			sdf += TVoxel::SDF_valueToFloat(localCopy[locId_parent + SDF_BLOCK_SIZE].sdf)*(float)localCopy[locId_parent + SDF_BLOCK_SIZE].w_depth;
			w_sum += localCopy[locId_parent + SDF_BLOCK_SIZE].w_depth;
		}
		if (doInterpolate.x&&doInterpolate.y) {
			sdf += TVoxel::SDF_valueToFloat(localCopy[locId_parent + SDF_BLOCK_SIZE + 1].sdf)*(float)localCopy[locId_parent + SDF_BLOCK_SIZE + 1].w_depth;
			w_sum += localCopy[locId_parent + SDF_BLOCK_SIZE + 1].w_depth;
		}
		if (doInterpolate.z) {
			locId_parent += SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;
			sdf += TVoxel::SDF_valueToFloat(localCopy[locId_parent].sdf)*(float)localCopy[locId_parent].w_depth;
			w_sum += localCopy[locId_parent].w_depth;
			if (doInterpolate.x) {
				sdf += TVoxel::SDF_valueToFloat(localCopy[locId_parent + 1].sdf)*(float)localCopy[locId_parent + 1].w_depth;
				w_sum += localCopy[locId_parent + 1].w_depth;
			}
			if (doInterpolate.y) {
				sdf += TVoxel::SDF_valueToFloat(localCopy[locId_parent + SDF_BLOCK_SIZE].sdf)*(float)localCopy[locId_parent + SDF_BLOCK_SIZE].w_depth;
				w_sum += localCopy[locId_parent + SDF_BLOCK_SIZE].w_depth;
			}
			if (doInterpolate.x&&doInterpolate.y) {
				sdf += TVoxel::SDF_valueToFloat(localCopy[locId_parent + SDF_BLOCK_SIZE + 1].sdf)*(float)localCopy[locId_parent + SDF_BLOCK_SIZE + 1].w_depth;
				w_sum += localCopy[locId_parent + SDF_BLOCK_SIZE + 1].w_depth;
			}
		}
		int fac = 1;
		if (doInterpolate.x) fac <<= 1;
		if (doInterpolate.y) fac <<= 1;
		if (doInterpolate.z) fac <<= 1;
		if (w_sum == 0) res.sdf = TVoxel::SDF_floatToValue(1.0f);
		else
		res.sdf = TVoxel::SDF_floatToValue(sdf / (float) (w_sum));
		res.w_depth = w_sum/fac;

		voxelBlock_child[locId] = res;
	}
}

template<class TVoxel>
void ITMVoxelBlockOpEngine_CUDA<TVoxel,ITMVoxelBlockHHash>::SplitVoxelBlocks(ITMScene<TVoxel,ITMVoxelBlockHHash> *scene, const ITMRenderState *renderState)
{
	const ITMRenderState_VH *renderState_vh = (const ITMRenderState_VH*)renderState;

	ITMHHashEntry *allHashEntries = scene->index.GetEntries();
	TVoxel *voxelBlocks = scene->localVBA.GetVoxelBlocks();
	int *voxelAllocationList = scene->localVBA.GetAllocationList();
	int *excessAllocationList = scene->index.GetExcessAllocationList();
	int &lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
	int *lastFreeExcessListIds = scene->index.GetLastFreeExcessListIds();

	const int *liveList = renderState_vh->GetVisibleEntryIDs();
	int liveListSize = renderState_vh->noVisibleEntries;

	ITMSafeCall(cudaMemset(blocklist_size_device, 0, sizeof(int)));

	dim3 blockSize(256);
	dim3 gridSize(ceil((float)liveListSize/(float)blockSize.x));

	if (liveListSize > 0) {
		ITMSafeCall(cudaMemcpy(noAllocatedVoxelEntries_device, &lastFreeVoxelBlockId, sizeof(int), cudaMemcpyHostToDevice));
		ITMSafeCall(cudaMemcpy(noAllocatedExcessEntries_device, lastFreeExcessListIds, ITMVoxelBlockHHash::noLevels * sizeof(int), cudaMemcpyHostToDevice));
		createSplits_device << <gridSize, blockSize >> >(liveList, liveListSize, complexities, allHashEntries, excessAllocationList, noAllocatedExcessEntries_device, voxelAllocationList, noAllocatedVoxelEntries_device, blocklist, blocklist_size_device);
		ITMSafeCall(cudaMemcpy(&lastFreeVoxelBlockId, noAllocatedVoxelEntries_device, sizeof(int), cudaMemcpyDeviceToHost));
		ITMSafeCall(cudaMemcpy(lastFreeExcessListIds, noAllocatedExcessEntries_device, ITMVoxelBlockHHash::noLevels * sizeof(int), cudaMemcpyDeviceToHost));
	}

	int blockListSize;
	ITMSafeCall(cudaMemcpy(&blockListSize, blocklist_size_device, sizeof(int), cudaMemcpyDeviceToHost));

	blockSize = dim3(SDF_BLOCK_SIZE, SDF_BLOCK_SIZE, SDF_BLOCK_SIZE);
	gridSize = dim3(blockListSize/8);
	if (blockListSize>0) {
		performSplitOperations_device << <gridSize, blockSize, SDF_BLOCK_SIZE3*sizeof(TVoxel) >> >(blocklist, voxelBlocks);
	}
}

__global__ void createMerges_device(int startOffset, int maxOffset, float *complexities, ITMHashEntry *allHashEntries, int *excessAllocationList, int *lastFreeExcessListIds, int *voxelAllocationList, int *lastFreeVoxelBlockId, int *blocklist, int *lastEntryBlockList)
{
	int htIdx = blockIdx.x * blockDim.x + threadIdx.x + startOffset;
	if (htIdx >= maxOffset) return;

	int blockId = allHashEntries[htIdx].ptr;
	// only merge, if the block was previously split
	if (blockId != -2) return;

	int parentLevel = ITMVoxelBlockHHash::GetLevelForEntry(htIdx);
	int childLevel = parentLevel-1;
	ITMHashEntry *childHashTable = &(allHashEntries[ITMVoxelBlockHHash::noTotalEntriesPerLevel * childLevel]);

	createMergeOperations(allHashEntries, childHashTable, excessAllocationList + childLevel * SDF_EXCESS_LIST_SIZE, lastFreeExcessListIds + childLevel, voxelAllocationList, lastFreeVoxelBlockId, complexities, blocklist, lastEntryBlockList, htIdx);
}

template<class TVoxel>
__global__ void performMergeOperations_device(const int *blocklist, TVoxel *voxelBlocks)
{
	extern __shared__ TVoxel localCopy[];

	int locId = threadIdx.x + threadIdx.y*SDF_BLOCK_SIZE + threadIdx.z*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;
	int blockListPos = blockIdx.x * 8;
	int parentBlockId = blocklist[blockListPos];

	// read data from old voxel blocks
	{
		Vector3i loc_child((threadIdx.x & 3)<<1, (threadIdx.y & 3)<<1, (threadIdx.z & 3)<<1);
		int child = ((threadIdx.x & 4)?1:0) | ((threadIdx.y & 4)?2:0) | ((threadIdx.z & 4)?4:0);
		int blockId_child = blocklist[blockListPos+child];
		TVoxel *voxelBlock_child = &(voxelBlocks[blockId_child * (SDF_BLOCK_SIZE3)]);
		// TODO: yes, I have heard of "aliasing effects", but I'm lazy
		localCopy[locId] = voxelBlock_child[loc_child.x + loc_child.y*SDF_BLOCK_SIZE + loc_child.z*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
	}
	__syncthreads();

	// copy data to the new parent block
	TVoxel *voxelBlock_parent = &(voxelBlocks[parentBlockId * (SDF_BLOCK_SIZE3)]);
	voxelBlock_parent[locId] = localCopy[locId];

	for (int child = 1; child < 8; ++child) {
		int blockId_child = blocklist[blockListPos+child];
		TVoxel *voxelBlock_child = &(voxelBlocks[blockId_child * (SDF_BLOCK_SIZE3)]);
		voxelBlock_child[locId] = TVoxel();
	}
}

template<class TVoxel>
void ITMVoxelBlockOpEngine_CUDA<TVoxel,ITMVoxelBlockHHash>::MergeVoxelBlocks(ITMScene<TVoxel,ITMVoxelBlockHHash> *scene, const ITMRenderState *renderState)
{
	ITMHashEntry *allHashEntries = scene->index.GetEntries();
	TVoxel *voxelBlocks = scene->localVBA.GetVoxelBlocks();
	int *voxelAllocationList = scene->localVBA.GetAllocationList();
	int *excessAllocationList = scene->index.GetExcessAllocationList();
	int &lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
	int *lastFreeExcessListIds = scene->index.GetLastFreeExcessListIds();

	ITMSafeCall(cudaMemset(blocklist_size_device, 0, sizeof(int)));

	dim3 blockSize(256);
	dim3 gridSize(ceil((float)(ITMVoxelBlockHHash::noTotalEntries - ITMVoxelBlockHHash::noTotalEntriesPerLevel) / (float)blockSize.x));


	ITMSafeCall(cudaMemcpy(noAllocatedVoxelEntries_device, &lastFreeVoxelBlockId, sizeof(int), cudaMemcpyHostToDevice));
	ITMSafeCall(cudaMemcpy(noAllocatedExcessEntries_device, lastFreeExcessListIds, ITMVoxelBlockHHash::noLevels * sizeof(int), cudaMemcpyHostToDevice));
	createMerges_device << <gridSize, blockSize >> >(ITMVoxelBlockHHash::noTotalEntriesPerLevel, ITMVoxelBlockHHash::noTotalEntries, complexities, allHashEntries, excessAllocationList, noAllocatedExcessEntries_device, voxelAllocationList, noAllocatedVoxelEntries_device, blocklist, blocklist_size_device);
	ITMSafeCall(cudaMemcpy(&lastFreeVoxelBlockId, noAllocatedVoxelEntries_device, sizeof(int), cudaMemcpyDeviceToHost));
	ITMSafeCall(cudaMemcpy(lastFreeExcessListIds, noAllocatedExcessEntries_device, ITMVoxelBlockHHash::noLevels * sizeof(int), cudaMemcpyDeviceToHost));

	int blockListSize;
	ITMSafeCall(cudaMemcpy(&blockListSize, blocklist_size_device, sizeof(int), cudaMemcpyDeviceToHost));

	blockSize = dim3(SDF_BLOCK_SIZE, SDF_BLOCK_SIZE, SDF_BLOCK_SIZE);
	gridSize = dim3(blockListSize/8);
	if (blockListSize>0) {
		performMergeOperations_device << <gridSize, blockSize, SDF_BLOCK_SIZE3*sizeof(TVoxel) >> >(blocklist, voxelBlocks);
	}
}

template<class TVoxel>
void ITMVoxelBlockOpEngine_CUDA<TVoxel,ITMVoxelBlockHHash>::SplitAndMerge(ITMScene<TVoxel,ITMVoxelBlockHHash> *scene, const ITMRenderState *renderState)
{
	ComputeComplexities(scene, renderState);
	SplitVoxelBlocks(scene, renderState);
	MergeVoxelBlocks(scene, renderState);
}

template class ITMVoxelBlockOpEngine_CUDA<ITMVoxel,ITMVoxelIndex>;
