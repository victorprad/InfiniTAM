// Copyright 2016 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMMeshingEngine_CUDA.h"

using namespace ITMLib;

__global__ void findAllocateBlocks(Vector4s *visibleBlockGlobalPos, const ITMHashEntry *hashTable, int noTotalEntries)
{
	int entryId = threadIdx.x + blockIdx.x * blockDim.x;
	if (entryId > noTotalEntries - 1) return;

	const ITMHashEntry &currentHashEntry = hashTable[entryId];

	if (currentHashEntry.ptr >= 0) 
		visibleBlockGlobalPos[currentHashEntry.ptr] = Vector4s(currentHashEntry.pos.x, currentHashEntry.pos.y, currentHashEntry.pos.z, 1);
}
