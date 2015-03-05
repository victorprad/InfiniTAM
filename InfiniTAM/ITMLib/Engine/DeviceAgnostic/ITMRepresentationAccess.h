// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMLibDefines.h"
#include "ITMPixelUtils.h"

#include "../../Utils/ITMMath.h"

template<typename T> _CPU_AND_GPU_CODE_ inline int hashIndex(const THREADPTR(T) & blockPos) {
	return (((uint)blockPos.x * 73856093u) ^ ((uint)blockPos.y * 19349669u) ^ ((uint)blockPos.z * 83492791u)) & (uint)SDF_HASH_MASK;
}

_CPU_AND_GPU_CODE_ inline int pointToVoxelBlockPos(const THREADPTR(Vector3i) & point, THREADPTR(Vector3i) &blockPos) {
	blockPos.x = ((point.x < 0) ? point.x - SDF_BLOCK_SIZE + 1 : point.x) / SDF_BLOCK_SIZE;
	blockPos.y = ((point.y < 0) ? point.y - SDF_BLOCK_SIZE + 1 : point.y) / SDF_BLOCK_SIZE;
	blockPos.z = ((point.z < 0) ? point.z - SDF_BLOCK_SIZE + 1 : point.z) / SDF_BLOCK_SIZE;
	return point.x + (point.y - blockPos.x) * SDF_BLOCK_SIZE + (point.z - blockPos.y) * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE - blockPos.z * SDF_BLOCK_SIZE3;
}

_CPU_AND_GPU_CODE_ inline int findVoxel(const CONSTANT(ITMLib::Objects::ITMVoxelBlockHash::IndexData) *voxelIndex, const THREADPTR(Vector3i) & point,
	THREADPTR(bool) &isFound, THREADPTR(ITMLib::Objects::ITMVoxelBlockHash::IndexCache) & cache)
{
	Vector3i blockPos;
	short linearIdx = pointToVoxelBlockPos(point, blockPos);

	if IS_EQUAL3(blockPos, cache.blockPos)
	{
		isFound = true;
		return cache.blockPtr + linearIdx;
	}

	int hashIdx = hashIndex(blockPos);

	while (true) 
	{
		ITMHashEntry hashEntry = voxelIndex[hashIdx];

		if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= 0)
		{
			isFound = true;
			cache.blockPos = blockPos; cache.blockPtr = hashEntry.ptr * SDF_BLOCK_SIZE3;
			return cache.blockPtr + linearIdx;
		}

		if (hashEntry.offset < 1) break;
		hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
	}

	isFound = false;
	return -1;
}

_CPU_AND_GPU_CODE_ inline int findVoxel(const CONSTANT(ITMLib::Objects::ITMVoxelBlockHash::IndexData) *voxelIndex, Vector3i point, THREADPTR(bool) &isFound)
{
	ITMLib::Objects::ITMVoxelBlockHash::IndexCache cache;
	return findVoxel(voxelIndex, point, isFound, cache);
}

_CPU_AND_GPU_CODE_ inline int findVoxel(const CONSTANT(ITMLib::Objects::ITMPlainVoxelArray::IndexData) *voxelIndex, const THREADPTR(Vector3i) & point_orig,
	THREADPTR(bool) &isFound)
{
	Vector3i point = point_orig - voxelIndex->offset;

	if ((point.x < 0) || (point.x >= voxelIndex->size.x) ||
	    (point.y < 0) || (point.y >= voxelIndex->size.y) ||
	    (point.z < 0) || (point.z >= voxelIndex->size.z)) {
		isFound = false;
		return -1;
	}

	int linearIdx = point.x + point.y * voxelIndex->size.x + point.z * voxelIndex->size.x * voxelIndex->size.y;

	isFound = true;
	return linearIdx;
}

_CPU_AND_GPU_CODE_ inline int findVoxel(const CONSTANT(ITMLib::Objects::ITMPlainVoxelArray::IndexData) *voxelIndex, const THREADPTR(Vector3i) & point_orig,
	THREADPTR(bool) &isFound, THREADPTR(ITMLib::Objects::ITMPlainVoxelArray::IndexCache) & cache)
{
	return findVoxel(voxelIndex, point_orig, isFound);
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const CONSTANT(TVoxel) *voxelData, const CONSTANT(ITMLib::Objects::ITMVoxelBlockHash::IndexData) *voxelIndex,
	const THREADPTR(Vector3i) & point, THREADPTR(bool) &isFound, THREADPTR(ITMLib::Objects::ITMVoxelBlockHash::IndexCache) & cache)
{
//	int voxelAddress = findVoxel(voxelIndex, point, isFound, cache);
//	return isFound ? voxelData[voxelAddress] : TVoxel();
	Vector3i blockPos;
	int linearIdx = pointToVoxelBlockPos(point, blockPos);

	if IS_EQUAL3(blockPos, cache.blockPos)
	{
		isFound = true;
		return voxelData[cache.blockPtr + linearIdx];
	}

	int hashIdx = hashIndex(blockPos);

	while (true) 
	{
		ITMHashEntry hashEntry = voxelIndex[hashIdx];

		if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= 0)
		{
			isFound = true;
			cache.blockPos = blockPos; cache.blockPtr = hashEntry.ptr * SDF_BLOCK_SIZE3;
			return voxelData[cache.blockPtr + linearIdx];
		}

		if (hashEntry.offset < 1) break;
		hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
	}

	isFound = false;
	return TVoxel();
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const CONSTANT(TVoxel) *voxelData, const CONSTANT(ITMLib::Objects::ITMVoxelBlockHash::IndexData) *voxelIndex,
	Vector3i point, THREADPTR(bool) &isFound)
{
	ITMLib::Objects::ITMVoxelBlockHash::IndexCache cache;
	return readVoxel(voxelData, voxelIndex, point, isFound, cache);
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const CONSTANT(TVoxel) *voxelData, const CONSTANT(ITMLib::Objects::ITMPlainVoxelArray::IndexData) *voxelIndex,
	const THREADPTR(Vector3i) & point_orig, THREADPTR(bool) &isFound)
{
	int voxelAddress = findVoxel(voxelIndex, point_orig, isFound);
	return isFound ? voxelData[voxelAddress] : TVoxel();
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const CONSTANT(TVoxel) *voxelData, const CONSTANT(ITMLib::Objects::ITMPlainVoxelArray::IndexData) *voxelIndex,
	const THREADPTR(Vector3i) & point_orig, THREADPTR(bool) &isFound, THREADPTR(ITMLib::Objects::ITMPlainVoxelArray::IndexCache) & cache)
{
	return readVoxel(voxelData, voxelIndex, point_orig, isFound);
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_uninterpolated(const CONSTANT(TVoxel) *voxelData,
	const CONSTANT(TIndex) *voxelIndex, Vector3f point, THREADPTR(bool) &isFound)
{
	TVoxel res = readVoxel(voxelData, voxelIndex, Vector3i((int)ROUND(point.x), (int)ROUND(point.y), (int)ROUND(point.z)), isFound);
	return TVoxel::SDF_valueToFloat(res.sdf);
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_uninterpolated(const CONSTANT(TVoxel) *voxelData,
	const CONSTANT(TIndex) *voxelIndex, Vector3f point, THREADPTR(bool) &isFound, THREADPTR(TCache) & cache)
{
	TVoxel res = readVoxel(voxelData, voxelIndex, Vector3i((int)ROUND(point.x), (int)ROUND(point.y), (int)ROUND(point.z)), isFound, cache);
	return TVoxel::SDF_valueToFloat(res.sdf);
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_interpolated(const CONSTANT(TVoxel) *voxelData,
	const CONSTANT(TIndex) *voxelIndex, Vector3f point, THREADPTR(bool) &isFound, THREADPTR(TCache) & cache)
{
	float res1, res2, v1, v2;
	Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);

	v1 = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), isFound, cache).sdf;
	v2 = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), isFound, cache).sdf;
	res1 = (1.0f - coeff.x) * v1 + coeff.x * v2;

	v1 = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), isFound, cache).sdf;
	v2 = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), isFound, cache).sdf;
	res1 = (1.0f - coeff.y) * res1 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);

	v1 = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), isFound, cache).sdf;
	v2 = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), isFound, cache).sdf;
	res2 = (1.0f - coeff.x) * v1 + coeff.x * v2;

	v1 = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), isFound, cache).sdf;
	v2 = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), isFound, cache).sdf;
	res2 = (1.0f - coeff.y) * res2 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);

	isFound = true;
	return TVoxel::SDF_valueToFloat((1.0f - coeff.z) * res1 + coeff.z * res2);
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline Vector4f readFromSDF_color4u_interpolated(const CONSTANT(TVoxel) *voxelData,
	const CONSTANT(TIndex) *voxelIndex, const THREADPTR(Vector3f) & point, THREADPTR(TCache) & cache)
{
	TVoxel resn; Vector3f ret = 0.0f; Vector4f ret4; bool isFound;
	Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), isFound, cache);
	ret += (1.0f - coeff.x) * (1.0f - coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), isFound, cache);
	ret += (coeff.x) * (1.0f - coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), isFound, cache);
	ret += (1.0f - coeff.x) * (coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), isFound, cache);
	ret += (coeff.x) * (coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), isFound, cache);
	ret += (1.0f - coeff.x) * (1.0f - coeff.y) * coeff.z * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), isFound, cache);
	ret += (coeff.x) * (1.0f - coeff.y) * coeff.z * resn.clr.toFloat();;

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), isFound, cache);
	ret += (1.0f - coeff.x) * (coeff.y) * coeff.z * resn.clr.toFloat();

	resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), isFound, cache);
	ret += (coeff.x) * (coeff.y) * coeff.z * resn.clr.toFloat();

	ret4.x = ret.x; ret4.y = ret.y; ret4.z = ret.z; ret4.w = 255.0f;

	return ret4 / 255.0f;
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline Vector3f computeSingleNormalFromSDF(const CONSTANT(TVoxel) *voxelData, const CONSTANT(TIndex) *voxelIndex, const THREADPTR(Vector3f) &point)
{
	bool isFound;

	Vector3f ret;
	Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);
	Vector3f ncoeff(1.0f - coeff.x, 1.0f - coeff.y, 1.0f - coeff.z);

	// all 8 values are going to be reused several times
	Vector4f front, back;
	front.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), isFound).sdf;
	front.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), isFound).sdf;
	front.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), isFound).sdf;
	front.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), isFound).sdf;
	back.x  = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), isFound).sdf;
	back.y  = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), isFound).sdf;
	back.z  = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), isFound).sdf;
	back.w  = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), isFound).sdf;

	Vector4f tmp;
	float p1, p2, v1;
	// gradient x
	p1 = front.x * ncoeff.y * ncoeff.z +
	     front.z *  coeff.y * ncoeff.z +
	     back.x  * ncoeff.y *  coeff.z +
	     back.z  *  coeff.y *  coeff.z;
	tmp.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 0, 0), isFound).sdf;
	tmp.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 1, 0), isFound).sdf;
	tmp.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 0, 1), isFound).sdf;
	tmp.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 1, 1), isFound).sdf;
	p2 = tmp.x * ncoeff.y * ncoeff.z +
	     tmp.y *  coeff.y * ncoeff.z +
	     tmp.z * ncoeff.y *  coeff.z +
	     tmp.w *  coeff.y *  coeff.z;
	v1 = p1 * coeff.x + p2 * ncoeff.x;

	p1 = front.y * ncoeff.y * ncoeff.z +
	     front.w *  coeff.y * ncoeff.z +
	     back.y  * ncoeff.y *  coeff.z +
	     back.w  *  coeff.y *  coeff.z;
	tmp.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(2, 0, 0), isFound).sdf;
	tmp.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(2, 1, 0), isFound).sdf;
	tmp.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(2, 0, 1), isFound).sdf;
	tmp.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(2, 1, 1), isFound).sdf;
	p2 = tmp.x * ncoeff.y * ncoeff.z +
	     tmp.y *  coeff.y * ncoeff.z +
	     tmp.z * ncoeff.y *  coeff.z +
	     tmp.w *  coeff.y *  coeff.z;

	ret.x = TVoxel::SDF_valueToFloat(p1 * ncoeff.x + p2 * coeff.x - v1);

	// gradient y
	p1 = front.x * ncoeff.x * ncoeff.z +
	     front.y *  coeff.x * ncoeff.z +
	     back.x  * ncoeff.x *  coeff.z +
	     back.y  *  coeff.x *  coeff.z;
	tmp.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, -1, 0), isFound).sdf;
	tmp.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, -1, 0), isFound).sdf;
	tmp.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, -1, 1), isFound).sdf;
	tmp.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, -1, 1), isFound).sdf;
	p2 = tmp.x * ncoeff.x * ncoeff.z +
	     tmp.y *  coeff.x * ncoeff.z +
	     tmp.z * ncoeff.x *  coeff.z +
	     tmp.w *  coeff.x *  coeff.z;
	v1 = p1 * coeff.y + p2 * ncoeff.y;

	p1 = front.z * ncoeff.x * ncoeff.z +
	     front.w *  coeff.x * ncoeff.z +
	     back.z  * ncoeff.x *  coeff.z +
	     back.w  *  coeff.x *  coeff.z;
	tmp.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 2, 0), isFound).sdf;
	tmp.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 2, 0), isFound).sdf;
	tmp.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 2, 1), isFound).sdf;
	tmp.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 2, 1), isFound).sdf;
	p2 = tmp.x * ncoeff.x * ncoeff.z +
	     tmp.y *  coeff.x * ncoeff.z +
	     tmp.z * ncoeff.x *  coeff.z +
	     tmp.w *  coeff.x *  coeff.z;

	ret.y = TVoxel::SDF_valueToFloat(p1 * ncoeff.y + p2 * coeff.y - v1);

	// gradient z
	p1 = front.x * ncoeff.x * ncoeff.y +
	     front.y *  coeff.x * ncoeff.y +
	     front.z * ncoeff.x *  coeff.y +
	     front.w *  coeff.x *  coeff.y;
	tmp.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, -1), isFound).sdf;
	tmp.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, -1), isFound).sdf;
	tmp.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, -1), isFound).sdf;
	tmp.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, -1), isFound).sdf;
	p2 = tmp.x * ncoeff.x * ncoeff.y +
	     tmp.y *  coeff.x * ncoeff.y +
	     tmp.z * ncoeff.x *  coeff.y +
	     tmp.w *  coeff.x *  coeff.y;
	v1 = p1 * coeff.z + p2 * ncoeff.z;

	p1 = back.x * ncoeff.x * ncoeff.y +
	     back.y *  coeff.x * ncoeff.y +
	     back.z * ncoeff.x *  coeff.y +
	     back.w *  coeff.x *  coeff.y;
	tmp.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 2), isFound).sdf;
	tmp.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 2), isFound).sdf;
	tmp.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 2), isFound).sdf;
	tmp.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 2), isFound).sdf;
	p2 = tmp.x * ncoeff.x * ncoeff.y +
	     tmp.y *  coeff.x * ncoeff.y +
	     tmp.z * ncoeff.x *  coeff.y +
	     tmp.w *  coeff.x *  coeff.y;

	ret.z = TVoxel::SDF_valueToFloat(p1 * ncoeff.z + p2 * coeff.z - v1);

	return ret;
}

typedef float (*round_funct_ptr)(float);

template<round_funct_ptr F_x, round_funct_ptr F_y, round_funct_ptr F_z>
class PointPosParser {
	public:
	_CPU_AND_GPU_CODE_ static inline int pointPosParse(const Vector3f & voxelPos, THREADPTR(Vector3i) &blockPos, int hierBlockSize) {
		Vector3i tmp((int)F_x(voxelPos.x/hierBlockSize), (int)F_y(voxelPos.y/hierBlockSize), (int)F_z(voxelPos.z/hierBlockSize));
		return pointToVoxelBlockPos(tmp, blockPos);
	}

	_CPU_AND_GPU_CODE_ static inline Vector3f pointPosRound(const THREADPTR(Vector3f) & voxelPos, int hierBlockSize) {
		return (Vector3i((int)F_x(voxelPos.x/hierBlockSize), (int)F_y(voxelPos.y/hierBlockSize), (int)F_z(voxelPos.z/hierBlockSize))*hierBlockSize).toFloat();
	}
};

typedef PointPosParser<floorf,floorf,floorf> PointPosFlooring;
typedef PointPosParser<roundf,roundf,roundf> PointPosRounding;

template<class TVoxel, class TRounding>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(DEVICEPTR(const TVoxel) *voxelData, DEVICEPTR(const ITMVoxelBlockHHash::IndexData) *voxelIndex, Vector3f & point, THREADPTR(bool) &isFound,
	THREADPTR(ITMVoxelBlockHHash::IndexCache) & cache)
{
	Vector3i blockPos;

	int linearIdx = TRounding::pointPosParse(point, blockPos, cache.blockSize);

	if (blockPos == cache.blockPos)
	{
		isFound = true; 
		point = TRounding::pointPosRound(point, cache.blockSize);
		return voxelData[cache.blockPtr + linearIdx];
	}

	int level = 0;
	isFound = false;
	while (true)
	{
		bool shouldContinueDown = false;
		int hierBlockSize = (1 << level);
		const ITMHHashEntry *hashTable = voxelIndex + level * ITMHHashTable::noTotalEntriesPerLevel;

		int linearIdx = TRounding::pointPosParse(point, blockPos, hierBlockSize);
		// start at ordered list of buckets
		int hashIdx = hashIndex(blockPos);

		while (true)
		{
			ITMHHashEntry hashEntry = hashTable[hashIdx];

			if (hashEntry.pos == blockPos)
			{
				if (hashEntry.ptr >= 0) {
					isFound = true;
					cache.blockPos = blockPos; cache.blockPtr = hashEntry.ptr * SDF_BLOCK_SIZE3; cache.blockSize = hierBlockSize;
					point = TRounding::pointPosRound(point, hierBlockSize);
					return voxelData[(hashEntry.ptr * SDF_BLOCK_SIZE3) + linearIdx];
				}
				
				if (hashEntry.ptr == -2) { shouldContinueDown = true; point = TRounding::pointPosRound(point, hierBlockSize); break; }
			}

			// crawl along excess list
			int offsetExcess = hashEntry.offset - 1;
			if (offsetExcess < 0) break;
			hashIdx = SDF_BUCKET_NUM + offsetExcess;
		}

		if (!shouldContinueDown) {
			level++;
			if (!(level < SDF_HASH_NO_H_LEVELS)) break;
		} else {
			level--;
			if (level < 0) break;
		}
	}

	point = TRounding::pointPosRound(point, (1<<(SDF_HASH_NO_H_LEVELS-1)));
	return TVoxel();
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_uninterpolated(DEVICEPTR(const TVoxel) *voxelData, DEVICEPTR(const ITMVoxelBlockHHash::IndexData) *voxelIndex, THREADPTR(Vector3f) point, THREADPTR(bool) &isFound,
	THREADPTR(ITMVoxelBlockHHash::IndexCache) & cache)
{
	return TVoxel::SDF_valueToFloat(readVoxel<TVoxel,PointPosRounding>(voxelData, voxelIndex, point, isFound, cache).sdf);
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_uninterpolated(DEVICEPTR(const TVoxel) *voxelData, DEVICEPTR(const ITMVoxelBlockHHash::IndexData) *voxelIndex, THREADPTR(Vector3f) point, THREADPTR(bool) &isFound)
{
	ITMVoxelBlockHHash::IndexCache cache;
	return readFromSDF_float_uninterpolated(voxelData, voxelIndex, point, isFound, cache);
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_interpolated(DEVICEPTR(const TVoxel) *voxelData, DEVICEPTR(const ITMVoxelBlockHHash::IndexData) *voxelIndex, Vector3f point, THREADPTR(bool) &isFound,
	THREADPTR(ITMVoxelBlockHHash::IndexCache) & cache)
{
	// horrendous case: ceilf==floorf...
	if (floorf(point.x) == ceilf(point.x)) point.x += 0.0001;
	if (floorf(point.y) == ceilf(point.y)) point.y += 0.0001;
	if (floorf(point.z) == ceilf(point.z)) point.z += 0.0001;

	float v[8];
	Vector3f points[8];
	char level;
	bool complicatedCase = false;
	points[0] = point;
	v[0] = readVoxel<TVoxel,PointPosParser<floorf,floorf,floorf> >(voxelData, voxelIndex, points[0], isFound, cache).sdf;
	if (!isFound) v[0] = TVoxel::SDF_valueToFloat(TVoxel::SDF_initialValue());
	bool tmpFound = isFound;
	level = cache.blockSize;

	points[1] = point;
	v[1] = readVoxel<TVoxel,PointPosParser<ceilf,floorf,floorf> >(voxelData, voxelIndex, points[1], isFound, cache).sdf;
	if (!isFound) v[1] = TVoxel::SDF_valueToFloat(TVoxel::SDF_initialValue());
	tmpFound |= isFound;
	if (level != cache.blockSize) complicatedCase = true;

	points[2] = point;
	v[2] = readVoxel<TVoxel,PointPosParser<floorf,ceilf,floorf> >(voxelData, voxelIndex, points[2], isFound, cache).sdf;
	if (!isFound) v[2] = TVoxel::SDF_valueToFloat(TVoxel::SDF_initialValue());
	tmpFound |= isFound;
	if (level != cache.blockSize) complicatedCase = true;

	points[3] = point;
	v[3] = readVoxel<TVoxel,PointPosParser<ceilf,ceilf,floorf> >(voxelData, voxelIndex, points[3], isFound, cache).sdf;
	if (!isFound) v[3] = TVoxel::SDF_valueToFloat(TVoxel::SDF_initialValue());
	tmpFound |= isFound;
	if (level != cache.blockSize) complicatedCase = true;

	points[4] = point;
	v[4] = readVoxel<TVoxel,PointPosParser<floorf,floorf,ceilf> >(voxelData, voxelIndex, points[4], isFound, cache).sdf;
	if (!isFound) v[4] = TVoxel::SDF_valueToFloat(TVoxel::SDF_initialValue());
	tmpFound |= isFound;
	if (level != cache.blockSize) complicatedCase = true;

	points[5] = point;
	v[5] = readVoxel<TVoxel,PointPosParser<ceilf,floorf,ceilf> >(voxelData, voxelIndex, points[5], isFound, cache).sdf;
	if (!isFound) v[5] = TVoxel::SDF_valueToFloat(TVoxel::SDF_initialValue());
	tmpFound |= isFound;
	if (level != cache.blockSize) complicatedCase = true;

	points[6] = point;
	v[6] = readVoxel<TVoxel,PointPosParser<floorf,ceilf,ceilf> >(voxelData, voxelIndex, points[6], isFound, cache).sdf;
	if (!isFound) v[6] = TVoxel::SDF_valueToFloat(TVoxel::SDF_initialValue());
	tmpFound |= isFound;
	if (level != cache.blockSize) complicatedCase = true;

	points[7] = point;
	v[7] = readVoxel<TVoxel,PointPosParser<ceilf,ceilf,ceilf> >(voxelData, voxelIndex, points[7], isFound, cache).sdf;
	if (!isFound) v[7] = TVoxel::SDF_valueToFloat(TVoxel::SDF_initialValue());
	isFound |= tmpFound;
	if (level != cache.blockSize) complicatedCase = true;

	float ret;
	if (!complicatedCase) {
		Vector3f coeff;
		(point / (float)level).toIntFloor(coeff);
		Vector3f ncoeff(1.0f - coeff.x, 1.0f - coeff.y, 1.0f - coeff.z);
		ret  = ncoeff.x * ncoeff.y * ncoeff.z * v[0];
		ret +=  coeff.x * ncoeff.y * ncoeff.z * v[1];
		ret += ncoeff.x *  coeff.y * ncoeff.z * v[2];
		ret +=  coeff.x *  coeff.y * ncoeff.z * v[3];
		ret += ncoeff.x * ncoeff.y *  coeff.z * v[4];
		ret +=  coeff.x * ncoeff.y *  coeff.z * v[5];
		ret += ncoeff.x *  coeff.y *  coeff.z * v[6];
		ret +=  coeff.x *  coeff.y *  coeff.z * v[7];
	} else {
		float A[7][7];
		const Vector3f & p0 = points[0];

		int r = 0;
		Vector3f tmp = points[r+1] - p0;
		A[r][0] = tmp.x*tmp.y*tmp.z; A[r][1] = tmp.y*tmp.z; A[r][2] = tmp.x*tmp.z; A[r][3] = tmp.z;
		A[r][4] = tmp.x*tmp.y; A[r][5] = tmp.y; A[r][6] = tmp.x;
		v[r+1] -= v[0];

		++r;
		tmp = points[r+1] - p0;
		A[r][0] = tmp.x*tmp.y*tmp.z; A[r][1] = tmp.y*tmp.z; A[r][2] = tmp.x*tmp.z; A[r][3] = tmp.z;
		A[r][4] = tmp.x*tmp.y; A[r][5] = tmp.y; A[r][6] = tmp.x;
		v[r+1] -= v[0];

		++r;
		tmp = points[r+1] - p0;
		A[r][0] = tmp.x*tmp.y*tmp.z; A[r][1] = tmp.y*tmp.z; A[r][2] = tmp.x*tmp.z; A[r][3] = tmp.z;
		A[r][4] = tmp.x*tmp.y; A[r][5] = tmp.y; A[r][6] = tmp.x;
		v[r+1] -= v[0];

		++r;
		tmp = points[r+1] - p0;
		A[r][0] = tmp.x*tmp.y*tmp.z; A[r][1] = tmp.y*tmp.z; A[r][2] = tmp.x*tmp.z; A[r][3] = tmp.z;
		A[r][4] = tmp.x*tmp.y; A[r][5] = tmp.y; A[r][6] = tmp.x;
		v[r+1] -= v[0];

		++r;
		tmp = points[r+1] - p0;
		A[r][0] = tmp.x*tmp.y*tmp.z; A[r][1] = tmp.y*tmp.z; A[r][2] = tmp.x*tmp.z; A[r][3] = tmp.z;
		A[r][4] = tmp.x*tmp.y; A[r][5] = tmp.y; A[r][6] = tmp.x;
		v[r+1] -= v[0];

		++r;
		tmp = points[r+1] - p0;
		A[r][0] = tmp.x*tmp.y*tmp.z; A[r][1] = tmp.y*tmp.z; A[r][2] = tmp.x*tmp.z; A[r][3] = tmp.z;
		A[r][4] = tmp.x*tmp.y; A[r][5] = tmp.y; A[r][6] = tmp.x;
		v[r+1] -= v[0];

		++r;
		tmp = points[r+1] - p0;
		A[r][0] = tmp.x*tmp.y*tmp.z; A[r][1] = tmp.y*tmp.z; A[r][2] = tmp.x*tmp.z; A[r][3] = tmp.z;
		A[r][4] = tmp.x*tmp.y; A[r][5] = tmp.y; A[r][6] = tmp.x;
		v[r+1] -= v[0];

		tmp = point - p0;

		float coeff[7];
		solveGaussianEliminationLower(&(A[0][0]), &(v[1]), &(coeff[0]), 7);

		ret = tmp.x*tmp.y*tmp.z * coeff[0] + tmp.y*tmp.z * coeff[1] + tmp.x*tmp.z * coeff[2] + tmp.z * coeff[3] +
		      tmp.x * tmp.y * coeff[4] + tmp.y * coeff[5] + tmp.x * coeff[6] + v[0];
	}

	return TVoxel::SDF_valueToFloat(ret);
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline Vector4f readFromSDF_color4u_interpolated(DEVICEPTR(const TVoxel) *voxelData, DEVICEPTR(const ITMVoxelBlockHHash::IndexData) *voxelIndex, Vector3f point, THREADPTR(ITMVoxelBlockHHash::IndexCache) & cache)
{
	// horrendous case: ceilf==floorf...
	if (floorf(point.x) == ceilf(point.x)) point.x += 0.0001;
	if (floorf(point.y) == ceilf(point.y)) point.y += 0.0001;
	if (floorf(point.z) == ceilf(point.z)) point.z += 0.0001;

	bool isFound;
	Vector3f v[8];
	Vector3f points[8];
	char level;
	bool complicatedCase = false;
	points[0] = point;
	v[0] = readVoxel<TVoxel,PointPosParser<floorf,floorf,floorf> >(voxelData, voxelIndex, points[0], isFound, cache).clr.toFloat();
	level = cache.blockSize;

	points[1] = point;
	v[1] = readVoxel<TVoxel,PointPosParser<ceilf,floorf,floorf> >(voxelData, voxelIndex, points[1], isFound, cache).clr.toFloat();
	if (level != cache.blockSize) complicatedCase = true;

	points[2] = point;
	v[2] = readVoxel<TVoxel,PointPosParser<floorf,ceilf,floorf> >(voxelData, voxelIndex, points[2], isFound, cache).clr.toFloat();
	if (level != cache.blockSize) complicatedCase = true;

	points[3] = point;
	v[3] = readVoxel<TVoxel,PointPosParser<ceilf,ceilf,floorf> >(voxelData, voxelIndex, points[3], isFound, cache).clr.toFloat();
	if (level != cache.blockSize) complicatedCase = true;

	points[4] = point;
	v[4] = readVoxel<TVoxel,PointPosParser<floorf,floorf,ceilf> >(voxelData, voxelIndex, points[4], isFound, cache).clr.toFloat();
	if (level != cache.blockSize) complicatedCase = true;

	points[5] = point;
	v[5] = readVoxel<TVoxel,PointPosParser<ceilf,floorf,ceilf> >(voxelData, voxelIndex, points[5], isFound, cache).clr.toFloat();
	if (level != cache.blockSize) complicatedCase = true;

	points[6] = point;
	v[6] = readVoxel<TVoxel,PointPosParser<floorf,ceilf,ceilf> >(voxelData, voxelIndex, points[6], isFound, cache).clr.toFloat();
	if (level != cache.blockSize) complicatedCase = true;

	points[7] = point;
	v[7] = readVoxel<TVoxel,PointPosParser<ceilf,ceilf,ceilf> >(voxelData, voxelIndex, points[7], isFound, cache).clr.toFloat();
	if (level != cache.blockSize) complicatedCase = true;

	Vector3f ret3;
	if (!complicatedCase) {
		Vector3f coeff;
		(point / (float)level).toIntFloor(coeff);
		Vector3f ncoeff(1.0f - coeff.x, 1.0f - coeff.y, 1.0f - coeff.z);
		ret3  = ncoeff.x * ncoeff.y * ncoeff.z * v[0];
		ret3 +=  coeff.x * ncoeff.y * ncoeff.z * v[1];
		ret3 += ncoeff.x *  coeff.y * ncoeff.z * v[2];
		ret3 +=  coeff.x *  coeff.y * ncoeff.z * v[3];
		ret3 += ncoeff.x * ncoeff.y *  coeff.z * v[4];
		ret3 +=  coeff.x * ncoeff.y *  coeff.z * v[5];
		ret3 += ncoeff.x *  coeff.y *  coeff.z * v[6];
		ret3 +=  coeff.x *  coeff.y *  coeff.z * v[7];
	} else {
		// Interpolation for Irregular Grid, see:
		// http://math.stackexchange.com/questions/828392/spatial-interpolation-for-irregular-grid
		float A[7][7];
		const Vector3f & p0 = points[0];

		int r = 0;
		Vector3f tmp = points[r+1] - p0;
		A[r][0] = tmp.x*tmp.y*tmp.z; A[r][1] = tmp.y*tmp.z; A[r][2] = tmp.x*tmp.z; A[r][3] = tmp.z;
		A[r][4] = tmp.x*tmp.y; A[r][5] = tmp.y; A[r][6] = tmp.x;
		v[r+1] -= v[0];

		++r;
		tmp = points[r+1] - p0;
		A[r][0] = tmp.x*tmp.y*tmp.z; A[r][1] = tmp.y*tmp.z; A[r][2] = tmp.x*tmp.z; A[r][3] = tmp.z;
		A[r][4] = tmp.x*tmp.y; A[r][5] = tmp.y; A[r][6] = tmp.x;
		v[r+1] -= v[0];

		++r;
		tmp = points[r+1] - p0;
		A[r][0] = tmp.x*tmp.y*tmp.z; A[r][1] = tmp.y*tmp.z; A[r][2] = tmp.x*tmp.z; A[r][3] = tmp.z;
		A[r][4] = tmp.x*tmp.y; A[r][5] = tmp.y; A[r][6] = tmp.x;
		v[r+1] -= v[0];

		++r;
		tmp = points[r+1] - p0;
		A[r][0] = tmp.x*tmp.y*tmp.z; A[r][1] = tmp.y*tmp.z; A[r][2] = tmp.x*tmp.z; A[r][3] = tmp.z;
		A[r][4] = tmp.x*tmp.y; A[r][5] = tmp.y; A[r][6] = tmp.x;
		v[r+1] -= v[0];

		++r;
		tmp = points[r+1] - p0;
		A[r][0] = tmp.x*tmp.y*tmp.z; A[r][1] = tmp.y*tmp.z; A[r][2] = tmp.x*tmp.z; A[r][3] = tmp.z;
		A[r][4] = tmp.x*tmp.y; A[r][5] = tmp.y; A[r][6] = tmp.x;
		v[r+1] -= v[0];

		++r;
		tmp = points[r+1] - p0;
		A[r][0] = tmp.x*tmp.y*tmp.z; A[r][1] = tmp.y*tmp.z; A[r][2] = tmp.x*tmp.z; A[r][3] = tmp.z;
		A[r][4] = tmp.x*tmp.y; A[r][5] = tmp.y; A[r][6] = tmp.x;
		v[r+1] -= v[0];

		++r;
		tmp = points[r+1] - p0;
		A[r][0] = tmp.x*tmp.y*tmp.z; A[r][1] = tmp.y*tmp.z; A[r][2] = tmp.x*tmp.z; A[r][3] = tmp.z;
		A[r][4] = tmp.x*tmp.y; A[r][5] = tmp.y; A[r][6] = tmp.x;
		v[r+1] -= v[0];

		tmp = point - p0;

		Vector3f coeff[7];
		solveGaussianEliminationLower(&(A[0][0]), &(v[1]), &(coeff[0]), 7);

		ret3 = tmp.x*tmp.y*tmp.z * coeff[0] + tmp.y*tmp.z * coeff[1] + tmp.x*tmp.z * coeff[2] + tmp.z * coeff[3] +
		      tmp.x * tmp.y * coeff[4] + tmp.y * coeff[5] + tmp.x * coeff[6] + v[0];
	}

	Vector4f ret4;
	ret4.x = ret3.x;
	ret4.y = ret3.y;
	ret4.z = ret3.z;
	ret4.w = 255.0f;
	return ret4 / 255.0f;
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline Vector3f computeSingleNormalFromSDF(DEVICEPTR(const TVoxel) *voxelData, DEVICEPTR(const ITMVoxelBlockHHash::IndexData) *voxelIndex, THREADPTR(Vector3f) pos)
{
	ITMVoxelBlockHHash::IndexCache cache;
	bool isFound;
	Vector3f pos_tmp = pos;
	readVoxel<TVoxel,PointPosFlooring>(voxelData, voxelIndex, pos_tmp, isFound, cache);
	int scale = cache.blockSize;

	Vector3f ret;
	float v1 = readFromSDF_float_interpolated(voxelData, voxelIndex, pos+Vector3f(-scale, 0.0f, 0.0f), isFound, cache);
	float v2 = readFromSDF_float_interpolated(voxelData, voxelIndex, pos+Vector3f(+scale, 0.0f, 0.0f), isFound, cache);
	ret.x = v2 - v1;
	v1 = readFromSDF_float_interpolated(voxelData, voxelIndex, pos+Vector3f(0.0f, -scale, 0.0f), isFound, cache);
	v2 = readFromSDF_float_interpolated(voxelData, voxelIndex, pos+Vector3f(0.0f, +scale, 0.0f), isFound, cache);
	ret.y = v2 - v1;
	v1 = readFromSDF_float_interpolated(voxelData, voxelIndex, pos+Vector3f(0.0f,0.0f, -scale), isFound, cache);
	v2 = readFromSDF_float_interpolated(voxelData, voxelIndex, pos+Vector3f(0.0f,0.0f, +scale), isFound, cache);
	ret.z = v2 - v1;

	return ret;
}

template<bool hasColor,class TVoxel,class TIndex> struct VoxelColorReader;

template<class TVoxel, class TIndex>
struct VoxelColorReader<false,TVoxel,TIndex> {
	_CPU_AND_GPU_CODE_ static Vector4f interpolate(const CONSTANT(TVoxel) *voxelData, const CONSTANT(typename TIndex::IndexData) *voxelIndex,
		const THREADPTR(Vector3f) & point)
	{ return Vector4f(0.0f,0.0f,0.0f,0.0f); }
};

template<class TVoxel, class TIndex>
struct VoxelColorReader<true,TVoxel,TIndex> {
	_CPU_AND_GPU_CODE_ static Vector4f interpolate(const CONSTANT(TVoxel) *voxelData, const CONSTANT(typename TIndex::IndexData) *voxelIndex,
		const THREADPTR(Vector3f) & point)
	{
		typename TIndex::IndexCache cache;
		return readFromSDF_color4u_interpolated(voxelData, voxelIndex, point, cache);
	}
};

