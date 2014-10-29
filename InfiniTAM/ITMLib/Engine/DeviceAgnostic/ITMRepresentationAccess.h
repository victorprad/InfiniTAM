// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMLibDefines.h"
#include "../../Utils/ITMPixelUtils.h"

template<typename T> _CPU_AND_GPU_CODE_ inline int hashIndex(const ITMLib::Vector3<T> voxelPos, const int hashMask) {
	return ((uint)(((uint)voxelPos.x * 73856093) ^ ((uint)voxelPos.y * 19349669) ^ ((uint)voxelPos.z * 83492791)) & (uint)hashMask);
}

_CPU_AND_GPU_CODE_ inline Vector3i pointToSDFBlock(Vector3i voxelPos) {
	if (voxelPos.x < 0) voxelPos.x -= SDF_BLOCK_SIZE - 1;
	if (voxelPos.y < 0) voxelPos.y -= SDF_BLOCK_SIZE - 1;
	if (voxelPos.z < 0) voxelPos.z -= SDF_BLOCK_SIZE - 1;
	return voxelPos / SDF_BLOCK_SIZE;
}

_CPU_AND_GPU_CODE_ inline int pointPosParse(Vector3i voxelPos, Vector3i &blockPos) {
	blockPos = pointToSDFBlock(voxelPos);
	Vector3i locPos = voxelPos - blockPos * SDF_BLOCK_SIZE;
	return locPos.x + locPos.y * SDF_BLOCK_SIZE + locPos.z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const TVoxel *voxelData, const ITMVoxelBlockHash::IndexData *voxelIndex, const Vector3i & point, bool &isFound,
	ITMVoxelBlockHash::IndexCache & cache)
{
	Vector3i blockPos; 
	int linearIdx = pointPosParse(point, blockPos);

	if (blockPos == cache.blockPos)
	{
		isFound = true; 
		return voxelData[cache.blockPtr + linearIdx];
	}

	const ITMHashEntry *hashTable = voxelIndex->entries_all;
	int offsetExcess = 0;

	int hashIdx = hashIndex(blockPos, SDF_HASH_MASK) * SDF_ENTRY_NUM_PER_BUCKET;

	isFound = false;

	//check ordered list
	for (int inBucketIdx = 0; inBucketIdx < SDF_ENTRY_NUM_PER_BUCKET; inBucketIdx++)
	{
		const ITMHashEntry &hashEntry = hashTable[hashIdx + inBucketIdx];
		offsetExcess = hashEntry.offset - 1;

		if (hashEntry.pos == blockPos && hashEntry.ptr >= 0)
		{
			isFound = true;
			cache.blockPos = blockPos; cache.blockPtr = hashEntry.ptr * SDF_BLOCK_SIZE3;
			return voxelData[(hashEntry.ptr * SDF_BLOCK_SIZE3) + linearIdx];
		}
	}

	//check excess list
	while (offsetExcess >= 0)
	{
		const ITMHashEntry &hashEntry = hashTable[SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + offsetExcess];

		if (hashEntry.pos == blockPos && hashEntry.ptr >= 0)
		{
			isFound = true;
			cache.blockPos = blockPos; cache.blockPtr = hashEntry.ptr * SDF_BLOCK_SIZE3;
			return voxelData[(hashEntry.ptr * SDF_BLOCK_SIZE3) + linearIdx];
		}

		offsetExcess = hashEntry.offset - 1;
	}

	return TVoxel();
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const TVoxel *voxelData, const ITMVoxelBlockHash::IndexData *voxelIndex, const Vector3i & point, bool &isFound)
{
	const ITMHashEntry *hashTable = voxelIndex->entries_all;
	Vector3i blockPos; int offsetExcess = 0;

	int linearIdx = pointPosParse(point, blockPos);
	int hashIdx = hashIndex(blockPos, SDF_HASH_MASK) * SDF_ENTRY_NUM_PER_BUCKET;

	isFound = false;

	//check ordered list
	for (int inBucketIdx = 0; inBucketIdx < SDF_ENTRY_NUM_PER_BUCKET; inBucketIdx++) 
	{
		const ITMHashEntry &hashEntry = hashTable[hashIdx + inBucketIdx];
		offsetExcess = hashEntry.offset - 1;

		if (hashEntry.pos == blockPos && hashEntry.ptr >= 0)
		{
			isFound = true;
			return voxelData[(hashEntry.ptr * SDF_BLOCK_SIZE3) + linearIdx];
		}
	}

	//check excess list
	while (offsetExcess >= 0)
	{
		const ITMHashEntry &hashEntry = hashTable[SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + offsetExcess];

		if (hashEntry.pos == blockPos && hashEntry.ptr >= 0)
		{
			isFound = true;
			return voxelData[(hashEntry.ptr * SDF_BLOCK_SIZE3) + linearIdx];
		}

		offsetExcess = hashEntry.offset - 1;
	}

	return TVoxel();
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const TVoxel *voxelData, const ITMPlainVoxelArray::IndexData *voxelIndex, const Vector3i & point_orig, bool &isFound)
{
	Vector3i point = point_orig - voxelIndex->offset;

	if ((point.x < 0) || (point.x >= voxelIndex->size.x) ||
	    (point.y < 0) || (point.y >= voxelIndex->size.y) ||
	    (point.z < 0) || (point.z >= voxelIndex->size.z)) {
		isFound = false;
		return TVoxel();
	}

	int linearIdx = point.x + point.y * voxelIndex->size.x + point.z * voxelIndex->size.x * voxelIndex->size.y;

	isFound = true;
	return voxelData[linearIdx];
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const TVoxel *voxelData, const ITMPlainVoxelArray::IndexData *voxelIndex, const Vector3i & point_orig, bool &isFound,
	ITMPlainVoxelArray::IndexCache & cache)
{
	return readVoxel(voxelData, voxelIndex, point_orig, isFound);
}

template<class TVoxel, class TAccess>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_uninterpolated(const TVoxel *voxelData, const TAccess *voxelIndex, Vector3f point, bool &isFound)
{
	TVoxel res = readVoxel(voxelData, voxelIndex, point.toIntRound(), isFound);
	return TVoxel::SDF_valueToFloat(res.sdf);
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_maybe_interpolate(const TVoxel *voxelData, const TIndex *voxelIndex, Vector3f point, bool &isFound, TCache & cache)
{
	Vector3i pos_round = point.toIntRound();
	TVoxel res = readVoxel(voxelData, voxelIndex, pos_round, isFound, cache);

	float ret = TVoxel::SDF_valueToFloat(res.sdf);
	if (fabsf(ret) > 0.25f) return ret;

	Vector3f coeff; Vector3i pos = point.toIntFloor(coeff);
	Vector3u skip(pos_round.x - pos.x, pos_round.y - pos.y, pos_round.z - pos.z);
	float c = (skip.x ? coeff.x : (1.0f-coeff.x)) * (skip.y ? coeff.y : (1.0f-coeff.y)) * (skip.z ? coeff.z : (1.0f-coeff.z));
	ret = c * res.sdf;

	Vector3i offs;
	for (offs.z = 0; offs.z < 2; ++offs.z) for (offs.y = 0; offs.y < 2; ++offs.y) for (offs.x = 0; offs.x < 2; ++offs.x) {
		if (offs.x == skip.x && offs.y == skip.y && offs.z == skip.z) continue;

		res = readVoxel(voxelData, voxelIndex, pos + offs, isFound, cache);

		c = (offs.x ? coeff.x : (1.0f-coeff.x)) * (offs.y ? coeff.y : (1.0f-coeff.y)) * (offs.z ? coeff.z : (1.0f-coeff.z));
		ret += res.sdf * c;
	}
	isFound = true;

	return TVoxel::SDF_valueToFloat(ret);
}

template<class TVoxel, class TAccess>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_maybe_interpolate(const TVoxel *voxelData, const TAccess *voxelIndex, Vector3f point, bool &isFound)
{
	Vector3i pos_round = point.toIntRound();
	TVoxel res = readVoxel(voxelData, voxelIndex, pos_round, isFound);

	float ret = TVoxel::SDF_valueToFloat(res.sdf);
	if (fabsf(ret) > 0.25f) return ret;

	Vector3f coeff; Vector3i pos = point.toIntFloor(coeff);
	Vector3u skip(pos_round.x - pos.x, pos_round.y - pos.y, pos_round.z - pos.z);
	float c = (skip.x ? coeff.x : (1.0f - coeff.x)) * (skip.y ? coeff.y : (1.0f - coeff.y)) * (skip.z ? coeff.z : (1.0f - coeff.z));
	ret = c * res.sdf;

	Vector3i offs;
	for (offs.z = 0; offs.z < 2; ++offs.z) for (offs.y = 0; offs.y < 2; ++offs.y) for (offs.x = 0; offs.x < 2; ++offs.x) {
		if (offs.x == skip.x && offs.y == skip.y && offs.z == skip.z) continue;

		res = readVoxel(voxelData, voxelIndex, pos + offs, isFound);

		c = (offs.x ? coeff.x : (1.0f - coeff.x)) * (offs.y ? coeff.y : (1.0f - coeff.y)) * (offs.z ? coeff.z : (1.0f - coeff.z));
		ret += res.sdf * c;
	}
	isFound = true;

	return TVoxel::SDF_valueToFloat(ret);
}

template<class TVoxel, class TAccess>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_interpolated(const TVoxel *voxelData, const TAccess *voxelIndex, Vector3f point, bool &isFound)
{
	TVoxel resn; float ret = 0;
	Vector3f coeff; Vector3i pos = point.toIntFloor(coeff);

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), isFound);
	if (!isFound) return TVoxel::SDF_valueToFloat(TVoxel::SDF_initialValue());
	ret += (1.0f - coeff.x) * (1.0f - coeff.y) * (1.0f - coeff.z) * (float)resn.sdf;

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), isFound);
	if (!isFound) return TVoxel::SDF_valueToFloat(TVoxel::SDF_initialValue());
	ret += (coeff.x) * (1.0f - coeff.y) * (1.0f - coeff.z) * (float)resn.sdf;

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), isFound);
	if (!isFound) return TVoxel::SDF_valueToFloat(TVoxel::SDF_initialValue());
	ret += (1.0f - coeff.x) * (coeff.y) * (1.0f - coeff.z) * (float)resn.sdf;

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), isFound);
	if (!isFound) return TVoxel::SDF_valueToFloat(TVoxel::SDF_initialValue());
	ret += (coeff.x) * (coeff.y) * (1.0f - coeff.z) * (float)resn.sdf;

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), isFound);
	if (!isFound) return TVoxel::SDF_valueToFloat(TVoxel::SDF_initialValue());
	ret += (1.0f - coeff.x) * (1.0f - coeff.y) * coeff.z * (float)resn.sdf;

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), isFound);
	if (!isFound) return TVoxel::SDF_valueToFloat(TVoxel::SDF_initialValue());
	ret += (coeff.x) * (1.0f - coeff.y) * coeff.z * (float)resn.sdf;

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), isFound);
	if (!isFound) return TVoxel::SDF_valueToFloat(TVoxel::SDF_initialValue());
	ret += (1.0f - coeff.x) * (coeff.y) * coeff.z * (float)resn.sdf;

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), isFound);
	if (!isFound) return TVoxel::SDF_valueToFloat(TVoxel::SDF_initialValue());
	ret += (coeff.x) * (coeff.y) * coeff.z * (float)resn.sdf;

	return TVoxel::SDF_valueToFloat(ret);
}

template<class TVoxel, class TAccess>
_CPU_AND_GPU_CODE_ inline Vector4f readFromSDF_color4u_interpolated(const TVoxel *voxelData, const TAccess *voxelIndex, const Vector3f & point)
{
	TVoxel resn; Vector3f ret = 0.0f; Vector4f ret4; bool isFound;
	Vector3f coeff; Vector3i pos = point.toIntFloor(coeff);

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), isFound);
	ret += (1.0f - coeff.x) * (1.0f - coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), isFound);
	ret += (coeff.x) * (1.0f - coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), isFound);
	ret += (1.0f - coeff.x) * (coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), isFound);
	ret += (coeff.x) * (coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), isFound);
	ret += (1.0f - coeff.x) * (1.0f - coeff.y) * coeff.z * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), isFound);
	ret += (coeff.x) * (1.0f - coeff.y) * coeff.z * resn.clr.toFloat();;

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), isFound);
	ret += (1.0f - coeff.x) * (coeff.y) * coeff.z * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), isFound);
	ret += (coeff.x) * (coeff.y) * coeff.z * resn.clr.toFloat();

	ret4.x = ret.x; ret4.y = ret.y; ret4.z = ret.z; ret4.w = 255.0f;

	return ret4 / 255.0f;
}
