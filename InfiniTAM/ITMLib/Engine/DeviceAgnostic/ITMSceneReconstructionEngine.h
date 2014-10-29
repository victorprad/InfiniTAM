// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMLibDefines.h"
#include "../../Utils/ITMPixelUtils.h"
#include "ITMRepresentationAccess.h"

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
	bool isFound;

	Vector3f ret;
	Vector3f coeff;  Vector3i pos = point.toIntFloor(coeff);
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
		lastFreeInBucketIdx = -1; bool foundValue = false; int offsetExcess = 0;
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
