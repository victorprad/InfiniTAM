// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMMultiMeshingEngine_Shared.h"
#include "../../../Objects/Scene/ITMMultiSceneAccess.h"

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline bool findPointNeighborsMulti(THREADPTR(Vector3f) *p, THREADPTR(float) *sdf, Vector3i blockLocation, const CONSTPTR(TVoxel) *localVBA, const CONSTPTR(TIndex) *hashTables, int hashTableIdx)
{
	int vmIndex; Vector3i localBlockLocation;

	ITMMultiCache cache;

	localBlockLocation = blockLocation + Vector3i(0, 0, 0);
	p[0] = hashTables->posesInv[hashTableIdx] * localBlockLocation.toFloat();
	sdf[0] = readFromSDF_float_interpolated(localVBA, hashTables, p[0], vmIndex, cache);
	if (!vmIndex || sdf[0] == 1.0f) return false;

	localBlockLocation = blockLocation + Vector3i(1, 0, 0);
	p[1] = hashTables->posesInv[hashTableIdx] * localBlockLocation.toFloat();
	sdf[1] = readFromSDF_float_interpolated(localVBA, hashTables, p[1], vmIndex, cache);
	if (!vmIndex || sdf[1] == 1.0f) return false;

	localBlockLocation = blockLocation + Vector3i(1, 1, 0);
	p[2] = hashTables->posesInv[hashTableIdx] * localBlockLocation.toFloat();
	sdf[2] = readFromSDF_float_interpolated(localVBA, hashTables, p[2], vmIndex, cache);
	if (!vmIndex || sdf[2] == 1.0f) return false;

	localBlockLocation = blockLocation + Vector3i(0, 1, 0);
	p[3] = hashTables->posesInv[hashTableIdx] * localBlockLocation.toFloat();
	sdf[3] = readFromSDF_float_interpolated(localVBA, hashTables, p[3], vmIndex, cache);
	if (!vmIndex || sdf[3] == 1.0f) return false;

	localBlockLocation = blockLocation + Vector3i(0, 0, 1);
	p[4] = hashTables->posesInv[hashTableIdx] * localBlockLocation.toFloat();
	sdf[4] = readFromSDF_float_interpolated(localVBA, hashTables, p[4], vmIndex, cache);
	if (!vmIndex || sdf[4] == 1.0f) return false;

	localBlockLocation = blockLocation + Vector3i(1, 0, 1);
	p[5] = hashTables->posesInv[hashTableIdx] * localBlockLocation.toFloat();
	sdf[5] = readFromSDF_float_interpolated(localVBA, hashTables, p[5], vmIndex, cache);
	if (!vmIndex || sdf[5] == 1.0f) return false;

	localBlockLocation = blockLocation + Vector3i(1, 1, 1);
	p[6] = hashTables->posesInv[hashTableIdx] * localBlockLocation.toFloat();
	sdf[6] = readFromSDF_float_interpolated(localVBA, hashTables, p[6], vmIndex, cache);
	if (!vmIndex || sdf[6] == 1.0f) return false;

	localBlockLocation = blockLocation + Vector3i(0, 1, 1);
	p[7] = hashTables->posesInv[hashTableIdx] * localBlockLocation.toFloat();
	sdf[7] = readFromSDF_float_interpolated(localVBA, hashTables, p[7], vmIndex, cache);
	if (!vmIndex || sdf[7] == 1.0f) return false;

	return true;
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline int buildVertListMulti(THREADPTR(Vector3f) *vertList, Vector3i globalPos, Vector3i localPos, const CONSTPTR(TVoxel) *localVBA, const CONSTPTR(TIndex) *hashTable, int hashTableIdx)
{
	Vector3f points[8]; float sdfVals[8];

	if (!findPointNeighborsMulti(points, sdfVals, globalPos + localPos, localVBA, hashTable, hashTableIdx)) return -1;

	int cubeIndex = 0;
	if (sdfVals[0] < 0) cubeIndex |= 1; if (sdfVals[1] < 0) cubeIndex |= 2;
	if (sdfVals[2] < 0) cubeIndex |= 4; if (sdfVals[3] < 0) cubeIndex |= 8;
	if (sdfVals[4] < 0) cubeIndex |= 16; if (sdfVals[5] < 0) cubeIndex |= 32;
	if (sdfVals[6] < 0) cubeIndex |= 64; if (sdfVals[7] < 0) cubeIndex |= 128;

	if (edgeTable[cubeIndex] == 0) return -1;

	if (edgeTable[cubeIndex] & 1) vertList[0] = sdfInterp(points[0], points[1], sdfVals[0], sdfVals[1]);
	if (edgeTable[cubeIndex] & 2) vertList[1] = sdfInterp(points[1], points[2], sdfVals[1], sdfVals[2]);
	if (edgeTable[cubeIndex] & 4) vertList[2] = sdfInterp(points[2], points[3], sdfVals[2], sdfVals[3]);
	if (edgeTable[cubeIndex] & 8) vertList[3] = sdfInterp(points[3], points[0], sdfVals[3], sdfVals[0]);
	if (edgeTable[cubeIndex] & 16) vertList[4] = sdfInterp(points[4], points[5], sdfVals[4], sdfVals[5]);
	if (edgeTable[cubeIndex] & 32) vertList[5] = sdfInterp(points[5], points[6], sdfVals[5], sdfVals[6]);
	if (edgeTable[cubeIndex] & 64) vertList[6] = sdfInterp(points[6], points[7], sdfVals[6], sdfVals[7]);
	if (edgeTable[cubeIndex] & 128) vertList[7] = sdfInterp(points[7], points[4], sdfVals[7], sdfVals[4]);
	if (edgeTable[cubeIndex] & 256) vertList[8] = sdfInterp(points[0], points[4], sdfVals[0], sdfVals[4]);
	if (edgeTable[cubeIndex] & 512) vertList[9] = sdfInterp(points[1], points[5], sdfVals[1], sdfVals[5]);
	if (edgeTable[cubeIndex] & 1024) vertList[10] = sdfInterp(points[2], points[6], sdfVals[2], sdfVals[6]);
	if (edgeTable[cubeIndex] & 2048) vertList[11] = sdfInterp(points[3], points[7], sdfVals[3], sdfVals[7]);

	return cubeIndex;
}