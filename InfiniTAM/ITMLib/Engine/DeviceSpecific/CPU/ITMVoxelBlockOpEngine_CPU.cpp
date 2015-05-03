// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMVoxelBlockOpEngine_CPU.h"
#include "../../DeviceAgnostic/ITMVoxelBlockOpEngine.h"
#include "../../../Objects/ITMRenderState_VH.h"

using namespace ITMLib::Engine;

template<class TVoxel>
ITMVoxelBlockOpEngine_CPU<TVoxel,ITMVoxelBlockHHash>::ITMVoxelBlockOpEngine_CPU(void) 
{
	complexities = (float*)malloc(sizeof(float) * ITMVoxelBlockHHash::noTotalEntries);
	blocklist = (int*)malloc(sizeof(int) * 8 * SDF_LOCAL_BLOCK_NUM);

	for (int i = 0; i < ITMVoxelBlockHHash::noTotalEntries; i++) complexities[i] = -1;
}

template<class TVoxel>
ITMVoxelBlockOpEngine_CPU<TVoxel,ITMVoxelBlockHHash>::~ITMVoxelBlockOpEngine_CPU(void) 
{
	free(blocklist);
	free(complexities);
}

template<class TVoxel>
void ITMVoxelBlockOpEngine_CPU<TVoxel,ITMVoxelBlockHHash>::ComputeComplexities(ITMScene<TVoxel,ITMVoxelBlockHHash> *scene, const ITMRenderState *renderState)
{
	const ITMRenderState_VH *renderState_vh = (const ITMRenderState_VH*) renderState;

	TVoxel *voxelBlocks = scene->localVBA.GetVoxelBlocks();
	ITMHashEntry *hashEntries = scene->index.GetEntries();

	const int *liveList = renderState_vh->GetVisibleEntryIDs();
	int liveListSize = renderState_vh->noVisibleEntries;

	for (int listIdx = 0; listIdx < liveListSize; listIdx++)
	{
		int htIdx = liveList[listIdx];
		int blockId = hashEntries[htIdx].ptr;

		const TVoxel *voxelBlock = &voxelBlocks[blockId * SDF_BLOCK_SIZE3];

		Vector3f X_sum(0.0f); Matrix3f XXT_sum(0.0f);

		for (int z = 0; z < SDF_BLOCK_SIZE - 1; z++) for (int y = 0; y < SDF_BLOCK_SIZE - 1; y++) for (int x = 0; x < SDF_BLOCK_SIZE - 1; x++)
		{
			Vector3f X(0.0f); Matrix3f XXT(0.0f);
			ComputePerVoxelSumAndCovariance(Vector3i(x, y, z), voxelBlock, X, XXT);
			X_sum += X; XXT_sum += XXT;
		}

		complexities[htIdx] = ComputeCovarianceDet(X_sum, XXT_sum);
	}
}

template<class TVoxel>
void performSplitOperations(int *blocklist, int blockListPos, TVoxel *voxelBlocks, TVoxel *localCopy)
{
	int parentBlockId = blocklist[blockListPos];
	TVoxel *voxelBlock_parent = &(voxelBlocks[parentBlockId * (SDF_BLOCK_SIZE3)]);
	// create backup
	for (int i = 0; i < SDF_BLOCK_SIZE3; ++i) localCopy[i] = voxelBlock_parent[i];

	// interpolation of SDF value, but nothing else
	// thread safe and seems to work reasonably well
	for (int child = 0; child < 8; ++child) {
		TVoxel *voxelBlock_child = &(voxelBlocks[blocklist[blockListPos++] * (SDF_BLOCK_SIZE3)]);

		for (int z=0; z<SDF_BLOCK_SIZE; ++z) for (int y=0; y<SDF_BLOCK_SIZE; ++y) for (int x=0; x<SDF_BLOCK_SIZE; ++x) {
			Vector3i loc_parent(x/2 + ((child&1)?4:0), y/2 + ((child&2)?4:0), z/2 + ((child&4)?4:0));
			Vector3u doInterpolate(x&1, y&1, z&1);
			if (loc_parent.x == 7) doInterpolate.x = 0;
			if (loc_parent.y == 7) doInterpolate.y = 0;
			if (loc_parent.z == 7) doInterpolate.z = 0;
			int locId_parent = loc_parent.x + loc_parent.y * SDF_BLOCK_SIZE + loc_parent.z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

			TVoxel res = localCopy[locId_parent];
			float sdf = res.sdf;

			if (doInterpolate.x) sdf += localCopy[locId_parent + 1].sdf;
			if (doInterpolate.y) sdf += localCopy[locId_parent + SDF_BLOCK_SIZE].sdf;
			if (doInterpolate.x&&doInterpolate.y) sdf += localCopy[locId_parent + SDF_BLOCK_SIZE + 1].sdf;
			if (doInterpolate.z) {
				locId_parent += SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;
				sdf += localCopy[locId_parent].sdf;
				if (doInterpolate.x) sdf += localCopy[locId_parent + 1].sdf;
				if (doInterpolate.y) sdf += localCopy[locId_parent + SDF_BLOCK_SIZE].sdf;
				if (doInterpolate.x&&doInterpolate.y) sdf += localCopy[locId_parent + SDF_BLOCK_SIZE + 1].sdf;
			}
			int fac = 1;
			if (doInterpolate.x) fac <<= 1;
			if (doInterpolate.y) fac <<= 1;
			if (doInterpolate.z) fac <<= 1;
			res.sdf = sdf / (float) fac;
			res.w_depth /= 2;

			int locId_child = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
			voxelBlock_child[locId_child] = res;
		}
	}
}

template<class TVoxel>
void ITMVoxelBlockOpEngine_CPU<TVoxel,ITMVoxelBlockHHash>::SplitVoxelBlocks(ITMScene<TVoxel,ITMVoxelBlockHHash> *scene, const ITMRenderState *renderState)
{
	const ITMRenderState_VH *renderState_vh = (const ITMRenderState_VH*) renderState;

	ITMHHashEntry *allHashEntries = scene->index.GetEntries();
	TVoxel *voxelBlocks = scene->localVBA.GetVoxelBlocks();
	int *voxelAllocationList = scene->localVBA.GetAllocationList();
	int *excessAllocationList = scene->index.GetExcessAllocationList();
	int *lastFreeVoxelBlockId = &(scene->localVBA.lastFreeBlockId);
	int *lastFreeExcessListIds = scene->index.GetLastFreeExcessListIds();

	const int *liveList = renderState_vh->GetVisibleEntryIDs();
	int liveListSize = renderState_vh->noVisibleEntries;

	int lastEntryBlockList = 0;

	for (int listIdx = 0; listIdx < liveListSize; ++listIdx)
	{
		int htIdx = liveList[listIdx];
		int parentLevel = ITMVoxelBlockHHash::GetLevelForEntry(htIdx);
		// finest level doesn't need splitting...
		if (parentLevel == 0) continue;

		if (complexities[htIdx] <= threshold_split) continue;
		complexities[htIdx] = -1;

		int childLevel = parentLevel-1;
		ITMHHashEntry *childHashTable = &(allHashEntries[ITMVoxelBlockHHash::noTotalEntriesPerLevel * childLevel]);
		int *childExcessAllocationList = excessAllocationList + (childLevel * SDF_EXCESS_LIST_SIZE);

		createSplitOperations(allHashEntries, childHashTable, childExcessAllocationList, lastFreeExcessListIds + childLevel, voxelAllocationList, lastFreeVoxelBlockId, blocklist, &lastEntryBlockList, htIdx, parentLevel);
	}

	TVoxel tempBlockMemory[SDF_BLOCK_SIZE3];
	for (int b = 0; b < lastEntryBlockList; b+=8) {
		performSplitOperations(blocklist, b, voxelBlocks, tempBlockMemory);
	}
}

template<class TVoxel>
void performMergeOperations(int *blocklist, int blockListPos, TVoxel *voxelBlocks, TVoxel *localCopy)
{
	int blockId_parent = blocklist[blockListPos];
	// read data from old voxel blocks
	for (int child = 0; child < 8; ++child) {
		int locId_parent = ((child&1)?4:0) + ((child&2)?4:0) * SDF_BLOCK_SIZE + ((child&4)?4:0) * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
		int blockId_child = blocklist[blockListPos+child];
		TVoxel *voxelBlock_child = &(voxelBlocks[blockId_child * (SDF_BLOCK_SIZE3)]);
		// TODO: yes, I have heard of "aliasing effects", but I'm lazy
		for (int z=0; z<SDF_BLOCK_SIZE/2; ++z) for (int y=0; y<SDF_BLOCK_SIZE/2; ++y) for (int x=0; x<SDF_BLOCK_SIZE/2; ++x) {
			localCopy[locId_parent + x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE] = voxelBlock_child[x*2 + y*2 * SDF_BLOCK_SIZE + z*2 * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE];
		}
	}

	// copy data to the new parent block
	TVoxel *voxelBlock_parent = &(voxelBlocks[blockId_parent * (SDF_BLOCK_SIZE3)]);
	for (int i = 0; i < (SDF_BLOCK_SIZE3); ++i) {
		voxelBlock_parent[i] = localCopy[i];
	}

	for (int child = 1; child < 8; ++child) {
		int blockId_child = blocklist[blockListPos+child];
		TVoxel *voxelBlock_child = &(voxelBlocks[blockId_child * (SDF_BLOCK_SIZE3)]);
		for (int i = 0; i < (SDF_BLOCK_SIZE3); ++i) {
			voxelBlock_child[i] = TVoxel();
		}
	}
}

template<class TVoxel>
void ITMVoxelBlockOpEngine_CPU<TVoxel,ITMVoxelBlockHHash>::MergeVoxelBlocks(ITMScene<TVoxel,ITMVoxelBlockHHash> *scene, const ITMRenderState *renderState)
{
	TVoxel *voxelBlocks = scene->localVBA.GetVoxelBlocks();
	ITMHHashEntry *allHashEntries = scene->index.GetEntries();
	int *voxelAllocationList = scene->localVBA.GetAllocationList();
	int *excessAllocationList = scene->index.GetExcessAllocationList();
	int *lastFreeVoxelBlockId = &(scene->localVBA.lastFreeBlockId);
	int *lastFreeExcessListIds = scene->index.GetLastFreeExcessListIds();

	int lastEntryBlockList = 0;

	// unfortunately we have to go through the whole list, as the -2 blocks
	// are not in the live list! :(
	for (int htIdx = ITMVoxelBlockHHash::noTotalEntriesPerLevel; htIdx < ITMVoxelBlockHHash::noTotalEntries; htIdx++)
	{
		int blockId = allHashEntries[htIdx].ptr;
		// only merge, if the block was previously split
		if (blockId != -2) continue;

		int parentLevel = ITMVoxelBlockHHash::GetLevelForEntry(htIdx);
		int childLevel = parentLevel-1;
		ITMHHashEntry *childHashTable = &(allHashEntries[ITMVoxelBlockHHash::noTotalEntriesPerLevel * childLevel]);

		createMergeOperations(allHashEntries, childHashTable, excessAllocationList + childLevel * SDF_EXCESS_LIST_SIZE, lastFreeExcessListIds + childLevel, voxelAllocationList, lastFreeVoxelBlockId, complexities, blocklist, &lastEntryBlockList, htIdx);
	}

	TVoxel tempBlockMemory[SDF_BLOCK_SIZE3];
	for (int b = 0; b < lastEntryBlockList; b+=8) {
		performMergeOperations(blocklist, b, voxelBlocks, tempBlockMemory);
	}
}

#if 0
static void checkHashTable(const ITMHHashEntry *allHashEntries)
{
	for (int htIdx = 0; htIdx < ITMVoxelBlockHHash::noTotalEntries; htIdx++) {
		int blockId = allHashEntries[htIdx].ptr;
		Vector3i blockPos_current(allHashEntries[htIdx].pos.x, allHashEntries[htIdx].pos.y, allHashEntries[htIdx].pos.z);

		int currentLevel = ITMVoxelBlockHHash::GetLevelForEntry(htIdx);
		int parentLevel = currentLevel+1;
		int childLevel = currentLevel-1;
		if (blockId == -2) {
			const ITMHHashEntry *childHashTable = &(allHashEntries[ITMVoxelBlockHHash::noTotalEntriesPerLevel * childLevel]);

			// check that all children exist
			if (currentLevel == 0) {
				fprintf(stderr, "split found on level 0\n");
			}

			for (int child = 0; child < 8; ++child) {
				Vector3i blockPos_child = blockPos_current * 2 + Vector3i(child&1, (child&2)?1:0, (child&4)?1:0);
				int hashIdx = hashIndex(blockPos_child);

				bool found = false;
				while (true) {
					const ITMHashEntry & hashEntry = childHashTable[hashIdx];

					if (hashEntry.pos == blockPos_child) {
						if ((hashEntry.ptr < 0)&&(hashEntry.ptr != -2)) {
							fprintf(stderr, "hash entry %i has child %i, but child has ptr %i\n", htIdx, hashIdx, hashEntry.ptr);
						} else found = true;
						break;
					}

					int offsetExcess = hashEntry.offset - 1;
					if (offsetExcess < 0) break;
					hashIdx = SDF_BUCKET_NUM + offsetExcess;
				}

				if (!found) {
					fprintf(stderr, "hash entry %i is missing a child at %i\n", htIdx, hashIdx);
				}
			}
		} else if ((blockId >= 0)&&(parentLevel < ITMVoxelBlockHHash::noLevels)) {
			// check that a parent exists
			const ITMHHashEntry *parentHashTable = &(allHashEntries[ITMVoxelBlockHHash::noTotalEntriesPerLevel * parentLevel]);

			Vector3i blockPos_parent(floor((float)blockPos_current.x/2.0f), floor((float)blockPos_current.y/2.0f), floor((float)blockPos_current.z/2.0f));
			int hashIdx = hashIndex(blockPos_parent);

			bool found = false;
			while (true) {
				const ITMHashEntry & hashEntry = parentHashTable[hashIdx];

				if (hashEntry.pos == blockPos_parent) {
					if (hashEntry.ptr != -2) {
						fprintf(stderr, "hash entry %i has parent %i, but parent has ptr %i\n", htIdx, hashIdx, hashEntry.ptr);
					} else found = true;
					break;
				}

				int offsetExcess = hashEntry.offset - 1;
				if (offsetExcess < 0) break;
				hashIdx = SDF_BUCKET_NUM + offsetExcess;
			}

			if (!found) {
				fprintf(stderr, "hash entry %i is missing a parent at %i\n", htIdx, hashIdx);
			}
		}
	}
}
#endif

template<class TVoxel>
void ITMVoxelBlockOpEngine_CPU<TVoxel,ITMVoxelBlockHHash>::SplitAndMerge(ITMScene<TVoxel,ITMVoxelBlockHHash> *scene, const ITMRenderState *renderState)
{
	ComputeComplexities(scene, renderState);
	SplitVoxelBlocks(scene, renderState);
	MergeVoxelBlocks(scene, renderState);
}

template class ITMLib::Engine::ITMVoxelBlockOpEngine_CPU<ITMVoxel,ITMVoxelIndex>;
