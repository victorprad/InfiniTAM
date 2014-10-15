// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMLibDefines.h"
#include "../../Utils/ITMPixelUtils.h"

template<typename T> _CPU_AND_GPU_CODE_ inline int hashIndex(const ITMLib::Vector3<T> vVoxPos, const int hashMask){
	return ((uint)(((uint)vVoxPos.x * 73856093) ^ ((uint)vVoxPos.y * 19349669) ^ ((uint)vVoxPos.z * 83492791)) & (uint)hashMask);
}

_CPU_AND_GPU_CODE_ inline Vector3i vVoxPosToSDFBlock(Vector3i vVoxPos){
	if (vVoxPos.x < 0) vVoxPos.x -= SDF_BLOCK_SIZE - 1;
	if (vVoxPos.y < 0) vVoxPos.y -= SDF_BLOCK_SIZE - 1;
	if (vVoxPos.z < 0) vVoxPos.z -= SDF_BLOCK_SIZE - 1;
	return vVoxPos / SDF_BLOCK_SIZE;
}

// Virtual voxel position local linearize
_CPU_AND_GPU_CODE_ inline int vVoxPosLinearize(Vector3i vLocVoxPos){
	return vLocVoxPos.x + vLocVoxPos.y * SDF_BLOCK_SIZE + vLocVoxPos.z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
}

// Parsing virtual voxel position
_CPU_AND_GPU_CODE_ inline int vVoxPosParse(Vector3i vVoxPos, Vector3i &blockPos){
	blockPos = vVoxPosToSDFBlock(vVoxPos);
	Vector3i locPos = vVoxPos - blockPos * SDF_BLOCK_SIZE;
	return vVoxPosLinearize(locPos);
}

template<class TVoxel, class TAccess>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const TVoxel *voxelData, const TAccess *voxelIndex, const Vector3i & point, bool &isFound);

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const TVoxel *voxelData, const ITMVoxelBlockHash::IndexData *voxelIndex, const Vector3i & point, bool &isFound)
{
	const ITMHashEntry *hashTable = voxelIndex->entries_all;
	const TVoxel *localVBA = voxelData;
	TVoxel result; Vector3i blockPos; int offsetExcess;

	int linearIdx = vVoxPosParse(point, blockPos);
	int hashIdx = hashIndex(blockPos, SDF_HASH_MASK) * SDF_ENTRY_NUM_PER_BUCKET;

	isFound = false;

	//check ordered list
	for (int inBucketIdx = 0; inBucketIdx < SDF_ENTRY_NUM_PER_BUCKET; inBucketIdx++) 
	{
		const ITMHashEntry &hashEntry = hashTable[hashIdx + inBucketIdx];
		offsetExcess = hashEntry.offset - 1;

		if (hashEntry.pos == blockPos && hashEntry.ptr >= 0)
		{
			result = localVBA[(hashEntry.ptr * SDF_BLOCK_SIZE3) + linearIdx];
			isFound = true;
			return result;
		}
	}

	//check excess list
	while (offsetExcess >= 0)
	{
		const ITMHashEntry &hashEntry = hashTable[SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + offsetExcess];

		if (hashEntry.pos == blockPos && hashEntry.ptr >= 0)
		{
			result = localVBA[(hashEntry.ptr * SDF_BLOCK_SIZE3) + linearIdx];
			isFound = true;
			return result;
		}

		offsetExcess = hashEntry.offset - 1;
	}

	return result;
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const TVoxel *voxelData, const ITMPlainVoxelArray::IndexData *voxelIndex, const Vector3i & point_orig, bool &isFound)
{
	TVoxel result;

	Vector3i point = point_orig - voxelIndex->offset;

	if ((point.x < 0) || (point.x >= voxelIndex->size.x) ||
	    (point.y < 0) || (point.y >= voxelIndex->size.y) ||
	    (point.z < 0) || (point.z >= voxelIndex->size.z)) {
		isFound = false;
		return result;
	}

	int linearIdx = point.x + point.y * voxelIndex->size.x + point.z * voxelIndex->size.x * voxelIndex->size.y;

	isFound = true;
	result = voxelData[linearIdx];
	return result;
}

template<class TVoxel, class TAccess>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_uninterpolated(const TVoxel *voxelData, const TAccess *voxelIndex, Vector3f point, bool &isFound)
{
	Vector3i pt_round = point.toIntRound(); 
	TVoxel res = readVoxel(voxelData, voxelIndex, pt_round, isFound);
	return TVoxel::SDF_valueToFloat(res.sdf);
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

template<bool hasColor,class TVoxel,class TAccess> struct VoxelColorReader;

template<class TVoxel, class TAccess>
struct VoxelColorReader<false,TVoxel,TAccess> {
	_CPU_AND_GPU_CODE_ static Vector4f interpolate(const TVoxel *voxelData, const TAccess *voxelIndex, const Vector3f & point)
	{ return Vector4f(0.0f,0.0f,0.0f,0.0f); }
};

template<class TVoxel, class TAccess>
struct VoxelColorReader<true,TVoxel,TAccess> {
	_CPU_AND_GPU_CODE_ static Vector4f interpolate(const TVoxel *voxelData, const TAccess *voxelIndex, const Vector3f & point)
	{ return readFromSDF_color4u_interpolated(voxelData, voxelIndex, point); }
};

template<class TVoxel, class TAccess>
_CPU_AND_GPU_CODE_ inline Vector3f computeSingleNormalFromSDF(const TVoxel *voxelData, const TAccess *voxelIndex, Vector3f point)
{
	//Vector3f ret; bool isFound;

	//float a, b, c, d, e, f;
	//a = readFromSDF_float_interpolated(hashTable, localVBA, point + Vector3f(0, 0, -1), isFound);
	//b = readFromSDF_float_interpolated(hashTable, localVBA, point + Vector3f(0, 0, 1), isFound);
	//c = readFromSDF_float_interpolated(hashTable, localVBA, point + Vector3f(0, -1, 0), isFound);
	//d = readFromSDF_float_interpolated(hashTable, localVBA, point + Vector3f(0, 1, 0), isFound);
	//e = readFromSDF_float_interpolated(hashTable, localVBA, point + Vector3f(-1, 0, 0), isFound);
	//f = readFromSDF_float_interpolated(hashTable, localVBA, point + Vector3f(1, 0, 0), isFound);

	//ret.x = f - e; ret.y = d - c; ret.z = b - a;

	//return ret;

	bool isFound;

	Vector3f coeff;  Vector3i pos = point.toIntFloor(coeff);

	Vector2f mid1front1row, mid2front1row, topfrontrow, botfrontrow, topbackrow, botbackrow, mid1back1row, mid2back1row;
	Vector4f mid1frontrow, mid2frontrow, mid1backrow, mid2backrow;

	Vector3f ret;

	mid1front1row.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, -1), isFound).sdf;
	mid1front1row.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, -1), isFound).sdf;

	mid2front1row.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, -1), isFound).sdf;
	mid2front1row.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, -1), isFound).sdf;

	topfrontrow.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, -1, 0), isFound).sdf;
	topfrontrow.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, -1, 0), isFound).sdf;

	mid1frontrow.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 0, 0), isFound).sdf;
	mid1frontrow.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), isFound).sdf;
	mid1frontrow.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), isFound).sdf;
	mid1frontrow.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(2, 0, 0), isFound).sdf;

	mid2frontrow.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 1, 0), isFound).sdf;
	mid2frontrow.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), isFound).sdf;
	mid2frontrow.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), isFound).sdf;
	mid2frontrow.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(2, 1, 0), isFound).sdf;

	botfrontrow.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 2, 0), isFound).sdf;
	botfrontrow.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 2, 0), isFound).sdf;

	topbackrow.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, -1, 1), isFound).sdf;
	topbackrow.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, -1, 1), isFound).sdf;

	mid1backrow.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 0, 1), isFound).sdf;
	mid1backrow.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), isFound).sdf;
	mid1backrow.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), isFound).sdf;
	mid1backrow.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(2, 0, 1), isFound).sdf;

	mid2backrow.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 1, 1), isFound).sdf;
	mid2backrow.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), isFound).sdf;
	mid2backrow.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), isFound).sdf;
	mid2backrow.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(2, 1, 1), isFound).sdf;

	botbackrow.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 2, 1), isFound).sdf;
	botbackrow.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 2, 1), isFound).sdf;

	mid1back1row.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 2), isFound).sdf;
	mid1back1row.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 2), isFound).sdf;

	mid2back1row.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 2), isFound).sdf;
	mid2back1row.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 2), isFound).sdf;

	Vector4f tmp1 = (1.0f - coeff.z) * mid1frontrow + coeff.z * mid1backrow;
	Vector4f tmp2 = (1.0f - coeff.z) * mid2frontrow + coeff.z * mid2backrow;

	ret.x = (1.0f - coeff.x) * (1.0f - coeff.y) * tmp1.z + (coeff.x) * (1.0f - coeff.y) * tmp1.w +
		(1.0f - coeff.x) * (coeff.y) * tmp2.z + (coeff.x) * (coeff.y) * tmp2.w;
	ret.x -= (1.0f - coeff.x) * (1.0f - coeff.y) * tmp1.x + (coeff.x) * (1.0f - coeff.y) * tmp1.y +
		(1.0f - coeff.x) * (coeff.y) * tmp2.x + (coeff.x) * (coeff.y) * tmp2.y;

	Vector2f tmp3 = (1.0f - coeff.z) * topfrontrow + coeff.z * topbackrow;
	Vector2f tmp4 = (1.0f - coeff.z) * botfrontrow + coeff.z * botbackrow;

	ret.y = (1.0f - coeff.x) * (1.0f - coeff.y) * tmp2.y + (coeff.x) * (1.0f - coeff.y) * tmp2.z +
		(1.0f - coeff.x) * (coeff.y) * tmp4.x + (coeff.x) * (coeff.y) * tmp4.y;
	ret.y -= (1.0f - coeff.x) * (1.0f - coeff.y) * tmp3.x + (coeff.x) * (1.0f - coeff.y) * tmp3.y +
		(1.0f - coeff.x) * (coeff.y) * tmp1.y + (coeff.x) * (coeff.y) * tmp1.z;

	tmp3 = (1.0f - coeff.y) * mid1front1row + coeff.y * mid2front1row;
	tmp1.x = (1.0f - coeff.y) * mid1frontrow.y + coeff.y * mid2frontrow.y; tmp1.y = (1.0f - coeff.y) * mid1frontrow.z + coeff.y * mid2frontrow.z;
	tmp2.x = (1.0f - coeff.y) * mid1backrow.y + coeff.y * mid2backrow.y; tmp2.y = (1.0f - coeff.y) * mid1backrow.z + coeff.y * mid2backrow.z;
	tmp4 = (1.0f - coeff.y) * mid1back1row + coeff.y * mid2back1row;

	ret.z = (1.0f - coeff.x) * (1.0f - coeff.z) * tmp2.x + (coeff.x) * (1.0f - coeff.z) * tmp2.y +
		(1.0f - coeff.x) * (coeff.z) * tmp4.x + (coeff.x) * (coeff.z) * tmp4.y;
	ret.z -= (1.0f - coeff.x) * (1.0f - coeff.z) * tmp3.x + (coeff.x) * (1.0f - coeff.z) * tmp3.y +
		(1.0f - coeff.x) * (coeff.z) * tmp1.x + (coeff.x) * (coeff.z) * tmp1.y;

	ret.x = TVoxel::SDF_valueToFloat(ret.x);
	ret.y = TVoxel::SDF_valueToFloat(ret.y);
	ret.z = TVoxel::SDF_valueToFloat(ret.z);

	return ret;
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline float computeUpdatedVoxelDepthInfo(TVoxel &voxel, const Vector4f & pt_model, const Matrix4f & M_d, const Vector4f & projParams_d,
	float mu, int maxW, const float *depth, const Vector2i & imgSize)
{
	Vector4f pt_camera; Vector2f pt_image;
	float depth_measure, eta, oldF, newF;
	int oldW, newW;

	// project point into image
	pt_camera = M_d * pt_model;
	if (pt_camera.z <= 0) return -1;

	pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
	pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
	if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return - 1;

	// get measured depth from image
	depth_measure = depth[(int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x];
	if (depth_measure <= 0.0) return -1;

	// check whether voxel needs updating
	eta = depth_measure - pt_camera.z;
	if (eta < -mu) return eta;

	// compute updated SDF value and reliability
	oldF = TVoxel::SDF_valueToFloat(voxel.sdf); oldW = voxel.w_depth;
	newF = MIN(1.0f, eta / mu);
	newW = 1;

	newF = oldW * oldF + newW * newF;
	newW = oldW + newW;
	newF /= newW;
	newW = MIN(newW, maxW);

	// write back
	voxel.sdf = TVoxel::SDF_floatToValue(newF);
	voxel.w_depth = newW;

	return eta;
}


template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void computeUpdatedVoxelColorInfo(TVoxel &voxel, const Vector4f & pt_model, const Matrix4f & M_rgb, const Vector4f & projParams_rgb,
	float mu, uchar maxW, float eta, const Vector4u *rgb, const Vector2i & imgSize)
{
	Vector4f pt_camera; Vector2f pt_image;
	Vector3f rgb_measure, oldC, newC; Vector3u buffV3u;
	float newW, oldW;

	buffV3u = voxel.clr;
	oldW = (float)voxel.w_color;

	oldC = buffV3u.toFloat() / 255.0f;
	newC = oldC;

	pt_camera = M_rgb * pt_model;

	pt_image.x = projParams_rgb.x * pt_camera.x / pt_camera.z + projParams_rgb.z;
	pt_image.y = projParams_rgb.y * pt_camera.y / pt_camera.z + projParams_rgb.w;

	if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return;

	rgb_measure = interpolateBilinear(rgb, pt_image, imgSize).toVector3() / 255.0f;
	//rgb_measure = rgb[(int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x].toVector3().toFloat() / 255.0f;
	newW = 1;

	newC = oldC * oldW + rgb_measure * newW;
	newW = oldW + newW;
	newC /= newW;
	newW = MIN(newW, maxW);

	buffV3u = (newC * 255.0f).toUChar();
	
	voxel.clr = buffV3u;
	voxel.w_color = (uchar)newW;
}

template<bool hasColor,class TVoxel> struct ComputeUpdatedVoxelInfo;

template<class TVoxel>
struct ComputeUpdatedVoxelInfo<false,TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(TVoxel & voxel, const Vector4f & pt_model,
		const Matrix4f & M_d, const Vector4f & projParams_d,
		const Matrix4f & M_rgb, const Vector4f & projParams_rgb,
		float mu, int maxW,
		const float *depth, const Vector2i & imgSize_d,
		const Vector4u *rgb, const Vector2i & imgSize_rgb)
	{
		computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, imgSize_d);
	}
};

template<class TVoxel>
struct ComputeUpdatedVoxelInfo<true,TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(TVoxel & voxel, const Vector4f & pt_model,
		const Matrix4f & M_d, const Vector4f & projParams_d,
		const Matrix4f & M_rgb, const Vector4f & projParams_rgb,
		float mu, int maxW,
		const float *depth, const Vector2i & imgSize_d,
		const Vector4u *rgb, const Vector2i & imgSize_rgb)
	{
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, imgSize_d);
		if ((eta > mu) || (fabsf(eta / mu) > 0.25f)) return;
		computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, maxW, eta, rgb, imgSize_rgb);
	}
};

_CPU_AND_GPU_CODE_ inline void buildHashAllocAndVisibleTypePP(uchar *entriesAllocType, uchar *entriesVisibleType, int x, int y, Vector3s *blockCoords,
	const float *depth, Matrix4f invM_d, Vector4f projParams_d, float mu, Vector2i imgSize, float oneOverVoxelSize, ITMHashEntry *hashTable,
	float viewFrustum_min, float viewFrustum_max)
{
	ITMHashEntry hashEntry;
	float depth_measure, direction_norm; unsigned int hashIdx; int noSteps, lastFreeInBucketIdx;
	Vector3f pt_camera_f, pt_block_s, pt_block_e, pt_block, direction; Vector3s pt_block_a;

	depth_measure = depth[x + y * imgSize.x];
	if (depth_measure <= 0 || (depth_measure - mu) < 0 || (depth_measure - mu) < viewFrustum_min || (depth_measure + mu) > viewFrustum_max) return;

	//find block coords for start ray
	pt_camera_f.z = depth_measure - mu;
	pt_camera_f.x = pt_camera_f.z * ((float(x) - projParams_d.z) * projParams_d.x);
	pt_camera_f.y = pt_camera_f.z * ((float(y) - projParams_d.w) * projParams_d.y);
	pt_block_s = (invM_d * pt_camera_f) * oneOverVoxelSize;

	//find block coords for end ray
	pt_camera_f.z = depth_measure + mu;
	pt_camera_f.x = pt_camera_f.z * ((float(x) - projParams_d.z) * projParams_d.x);
	pt_camera_f.y = pt_camera_f.z * ((float(y) - projParams_d.w) * projParams_d.y);
	pt_block_e = (invM_d * pt_camera_f) * oneOverVoxelSize;

	direction = pt_block_e - pt_block_s;
	direction_norm = 1.0f / sqrtf(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
	direction *= direction_norm;

	pt_block = pt_block_s - direction;

	noSteps = (int)ceilf(1.0f / direction_norm) + 1;

	//add neighbouring blocks
	for (int i = 0; i < noSteps; i++)
	{
		pt_block_a = pt_block.toShortFloor();

		//compute index in hash table
		hashIdx = hashIndex(pt_block_a, SDF_HASH_MASK) * SDF_ENTRY_NUM_PER_BUCKET;

		//check if hash table contains entry
		lastFreeInBucketIdx = -1; bool foundValue = false; int offsetExcess;
		for (int inBucketIdx = 0; inBucketIdx < SDF_ENTRY_NUM_PER_BUCKET; inBucketIdx++)
		{
			const ITMHashEntry &hashEntry = hashTable[hashIdx + inBucketIdx];
			offsetExcess = hashEntry.offset - 1;

			if (hashEntry.pos == pt_block_a && hashEntry.ptr >= -1)
			{
				if (hashEntry.ptr == -1) entriesVisibleType[hashIdx + inBucketIdx] = 2;
				else entriesVisibleType[hashIdx + inBucketIdx] = 1;

				foundValue = true;
				break;
			}

			if (lastFreeInBucketIdx == -1 && hashEntry.ptr < -1) lastFreeInBucketIdx = inBucketIdx;
		}

		if (!foundValue)
		{
			int hashIdx_toModify; //will contain parent index for excess list or normal hash+bucket index for ordered list

			if (lastFreeInBucketIdx >= 0) //not found and have room in the ordered part of the list (-> no excess list to search)
			{
				hashIdx_toModify = hashIdx + lastFreeInBucketIdx;

				entriesAllocType[hashIdx_toModify] = 1; //needs allocation and has room in ordered list
				entriesVisibleType[hashIdx_toModify] = 1; //new entry is visible

				blockCoords[hashIdx_toModify] = pt_block_a; //per-image hash collisions are ignored (will be picked up next frame)
			}
			else //might be in the excess list
			{
				hashIdx_toModify = hashIdx + SDF_ENTRY_NUM_PER_BUCKET - 1;

				int noOrderedEntries = SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET;

				while (offsetExcess >= 0)
				{
					const ITMHashEntry &hashEntry = hashTable[noOrderedEntries + offsetExcess];

					if (hashEntry.pos == pt_block_a && hashEntry.ptr >= -1)
					{
						if (hashEntry.ptr == -1) entriesVisibleType[noOrderedEntries + offsetExcess] = 2;
						else entriesVisibleType[noOrderedEntries + offsetExcess] = 1;

						foundValue = true;
						break;
					}

					hashIdx_toModify = noOrderedEntries + offsetExcess;
					offsetExcess = hashEntry.offset - 1;
				}

				if (!foundValue) //still not found -> must add into excess list
				{
					entriesAllocType[hashIdx_toModify] = 2; //needs allocation in the excess list
					blockCoords[hashIdx_toModify] = pt_block_a; //per-image hash collisions are ignored 
				}
			}
		}

		pt_block += direction;
	}
}
