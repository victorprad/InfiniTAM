// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMLibDefines.h"
#include "../../Utils/ITMHelpers.h"
#include "ITMRepresentationAccess.h"

static const float threshold_split = 0.0001f;
static const float threshold_merge = 0.0001f;

// if a hash entry has this value for the block ptr, it's considered empty
static const int EMPTY_BLOCK_PTR = -3;
// if a hash entry has this value for the offset, it's considererd to be at the end of the linked list
static const int EMPTY_LINKED_LIST = 0;

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void ComputePerVoxelSumAndCovariance(Vector3i loc, const TVoxel* voxelBlock, Vector3f &X, Matrix3f &XXT)
{
	int locId = loc.x + loc.y * SDF_BLOCK_SIZE + loc.z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

	float sdfLocId = TVoxel::SDF_valueToFloat(voxelBlock[locId].sdf);

	X.x = sdfLocId - TVoxel::SDF_valueToFloat(voxelBlock[locId + 1].sdf);
	X.y = sdfLocId - TVoxel::SDF_valueToFloat(voxelBlock[locId + SDF_BLOCK_SIZE].sdf);
	X.z = sdfLocId - TVoxel::SDF_valueToFloat(voxelBlock[locId + SDF_BLOCK_SIZE * SDF_BLOCK_SIZE].sdf);

	float Xnorm = sqrtf((float)(X.x * X.x + X.y * X.y + X.z * X.z));
	X = (fabsf(Xnorm) > 0) ? (X / Xnorm) : X;

	XXT.m00 = X.x * X.x; XXT.m01 = X.x * X.y; XXT.m02 = X.x * X.z;
	XXT.m10 = X.y * X.x; XXT.m11 = X.y * X.y; XXT.m12 = X.y * X.z;
	XXT.m20 = X.z * X.x; XXT.m21 = X.z * X.y; XXT.m22 = X.z * X.z;
}

_CPU_AND_GPU_CODE_ inline float ComputeCovarianceDet(Vector3f X_sum, Matrix3f XXT_sum)
{
	X_sum *= 1.0f / (float)SDF_BLOCK_SIZE3;
	XXT_sum *= 1.0f / (float)SDF_BLOCK_SIZE3;

	XXT_sum.m00 -= X_sum.x * X_sum.x; XXT_sum.m01 -= X_sum.x * X_sum.y; XXT_sum.m02 -= X_sum.x * X_sum.z;
	XXT_sum.m10 -= X_sum.y * X_sum.x; XXT_sum.m11 -= X_sum.y * X_sum.y; XXT_sum.m12 -= X_sum.y * X_sum.z;
	XXT_sum.m20 -= X_sum.z * X_sum.x; XXT_sum.m21 -= X_sum.z * X_sum.y; XXT_sum.m22 -= X_sum.z * X_sum.z;

	return XXT_sum.det();
}

/** This function updates the hash table to perform a block split. It 
    allocates all the new blocks in the hash table along with new blocks in
    the voxel block array. The main output is the "blocklist", which has groups
    of 8 block indices, the first of each being a parent block, the other 7 are
    empty, newly allocated voxel blocks for the children.
*/
_CPU_AND_GPU_CODE_ inline void createSplitOperations(ITMHashEntry *hashTableParent, ITMHashEntry *hashTableChild,
	int *excessAllocationList, int *lastFreeExcessListId,
	int *voxelAllocationList, int *lastFreeVoxelBlockId,
	int *blocklist, int *lastEntryBlockList, int htIdx_parent, int parentLevel)
{
	int blockId_parent = hashTableParent[htIdx_parent].ptr;

	Vector3i blockPos_parent(hashTableParent[htIdx_parent].pos.x, hashTableParent[htIdx_parent].pos.y, hashTableParent[htIdx_parent].pos.z);

	// allocate:
	// - get 7 new voxel block ptr (atomic)
	int newBlockListPtr = myAtomicSub(lastFreeVoxelBlockId, 7);
	int newBlockPtr = blockId_parent;

	// any space left in voxel block array?
	if (newBlockListPtr < 7) {
		// if it fails for this one, all other parallel ones will
		// equally fail, so this is thread safe:
		*lastFreeVoxelBlockId = newBlockListPtr;
		return;
	}

	int posInBlockList = myAtomicAdd(lastEntryBlockList, 8);

	// - find children, for each of them
	for (int child = 0; child < 8; ++child) {
		blocklist[posInBlockList++] = newBlockPtr;

		// - compute hash
		Vector3i blockPos_child = blockPos_parent * 2 + Vector3i(child&1, (child&2)?1:0, (child&4)?1:0);
		int hashIdx = hashIndex(blockPos_child, SDF_HASH_MASK);

		bool allocated = false;
		// - go through linked list
		while (true) {
			ITMHashEntry & hashEntry = hashTableChild[hashIdx];

			// - atomic_cas: if ptr == -1 then ptr = new ptr. if success, done
			int old = myAtomicCAS(&(hashEntry.ptr), EMPTY_BLOCK_PTR, newBlockPtr);
			if (old == EMPTY_BLOCK_PTR) {
				allocated = true;
				break;
			}

			if (hashEntry.offset == EMPTY_LINKED_LIST) break;
			hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
		}
		// - if no space in allocated excess list
		if (!allocated) {
			// - get new space in excess list
			int newOffsetExcess = myAtomicSub(lastFreeExcessListId, 1);
			// any space left in excess list?
			if (newOffsetExcess < 0) {
				// TODO: deallocate voxel block
				// this is nasty, because someone else might try to allocate one at the same time
				continue;
			}
			newOffsetExcess = excessAllocationList[newOffsetExcess];

			// - prepare the entry
			hashTableChild[SDF_BUCKET_NUM + newOffsetExcess].offset = EMPTY_LINKED_LIST;
			hashTableChild[SDF_BUCKET_NUM + newOffsetExcess].ptr = newBlockPtr;

			// - start from end of ordered list:
			while (true) {
				ITMHashEntry & hashEntry = hashTableChild[hashIdx];

				// - atomic_cas: if offsetExcess == -1 then write new ptr. done
				int old = myAtomicCAS(&hashEntry.offset, EMPTY_LINKED_LIST, newOffsetExcess + 1);
				if (old == EMPTY_LINKED_LIST) {
					allocated = true;
					break;
				}
				// - if failed, go to next entry
				hashIdx = SDF_BUCKET_NUM + old - 1;
			}
			hashIdx = SDF_BUCKET_NUM + newOffsetExcess;
		}
		// make sure the allocation worked well
		if (!allocated) continue;

		// write data to new hash entry
		hashTableChild[hashIdx].pos.x = blockPos_child.x;
		hashTableChild[hashIdx].pos.y = blockPos_child.y;
		hashTableChild[hashIdx].pos.z = blockPos_child.z;

		newBlockPtr = voxelAllocationList[newBlockListPtr--];
	}

	// write -2 to parent block ptr
	hashTableParent[htIdx_parent].ptr = -2;
}

_CPU_AND_GPU_CODE_ inline void createMergeOperations(ITMHashEntry *hashTableParent, ITMHashEntry *hashTableChild, int *excessAllocationList, int *lastFreeExcessListId, int *voxelAllocationList, int *lastFreeVoxelBlockId, float *complexities, int *blocklist, int *lastEntryBlockList, int htIdx_parent)
{
	Vector3i blockPos_parent(hashTableParent[htIdx_parent].pos.x, hashTableParent[htIdx_parent].pos.y, hashTableParent[htIdx_parent].pos.z);

	// - find children
	int childPredecessors[8];
	int childPositions[8];
	for (int child = 0; child < 8; ++child) {
		// - compute hash
		Vector3i blockPos_child = blockPos_parent * 2 + Vector3i(child&1, (child&2)?1:0, (child&4)?1:0);
		int hashIdx = hashIndex(blockPos_child, SDF_HASH_MASK);

		// - go through linked list
		int predecessor = -1;
		while (true) {
			const ITMHashEntry & hashEntry = hashTableChild[hashIdx];
			if (hashEntry.pos == blockPos_child) {
				childPositions[child] = hashIdx;
				break;
			}
			predecessor = hashIdx;
			int offsetExcess = hashEntry.offset - 1;
			if (offsetExcess < 0) break;
			hashIdx = SDF_BUCKET_NUM + offsetExcess;
		}
		childPredecessors[child] = predecessor;
	}

	// decide whether or not to merge
	bool shouldMerge = true;
	for (int child = 0; child < 8; ++child) {
		const ITMHashEntry & hashEntry = hashTableChild[childPositions[child]];
		if (hashEntry.ptr < 0) {
			shouldMerge = false;
			break;
		}

		if ((complexities[hashEntry.ptr] > threshold_merge)||
		    (complexities[hashEntry.ptr] < 0.0f)) {
			shouldMerge = false;
			break;
		}
	}

	if (!shouldMerge) return;

	// merge:

	// - write vba ptr to hash entry of the parent
	hashTableParent[htIdx_parent].ptr = hashTableChild[childPositions[0]].ptr;

	int oldBlockListPtr = myAtomicAdd(lastFreeVoxelBlockId, 7)+1;
	int posInBlockList = myAtomicAdd(lastEntryBlockList, 8);

	// - for each child
	for (int child = 0; child < 8; ++child) {
		ITMHashEntry & hashEntry = hashTableChild[childPositions[child]];
		complexities[hashEntry.ptr] = -1;

		// - release voxel block
		if (child > 0) voxelAllocationList[oldBlockListPtr++] = hashEntry.ptr;
		blocklist[posInBlockList++] = hashEntry.ptr;

		//   - set vba ptr in hash table to -1
		hashEntry.ptr = EMPTY_BLOCK_PTR;

		// - (optional) clean up excess list...
		if (childPredecessors[child] == -1) continue;
		if (hashEntry.offset != 0) continue; // this will avoid race conditions! However, it will not clean up the whole excess list...
		hashTableChild[childPredecessors[child]].offset = 0;
		int place = myAtomicAdd(lastFreeExcessListId) + 1;
		excessAllocationList[place] = childPositions[child] - SDF_BUCKET_NUM;
	}
}

