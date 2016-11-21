// Copyright 2016 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMRepresentationAccess.h"

#define MAX_NUM_SCENES 32

namespace ITMLib {

	struct ITMMultiCache {};

	template<class TIndex>
	class ITMMultiIndex {
	public:
		typedef TIndex IndexType;
		struct IndexData {
			typedef TIndex IndexType;
			int numScenes;
			typename TIndex::IndexData *index[MAX_NUM_SCENES];
			Matrix4f poses_vs[MAX_NUM_SCENES];
			Matrix4f posesInv[MAX_NUM_SCENES];
		};
		typedef ITMMultiCache IndexCache;
	};

	template<class TVoxel>
	class ITMMultiVoxel {
	public:
		typedef TVoxel VoxelType;
		TVoxel* voxels[MAX_NUM_SCENES];

		static const CONSTPTR(bool) hasColorInformation = TVoxel::hasColorInformation;
	};
}

template<class TMultiVoxel, class TMultiIndex>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_uninterpolated(const TMultiVoxel *voxelData, const TMultiIndex *voxelIndex, const Vector3f & point, bool & hash_found, ITMLib::ITMMultiCache & _cache)
{
	typedef typename TMultiVoxel::VoxelType TVoxel;
	typedef typename TMultiIndex::IndexType TIndex;

	float sum_sdf = 0.0f;
	int sum_weights = 0;
	hash_found = false;
	for (int sceneId = 0; sceneId < voxelIndex->numScenes; ++sceneId) {
		Vector3f point_local = voxelIndex->poses_vs[sceneId] * point;

		bool isFound;
		typename TIndex::IndexCache cache;
		const TVoxel & v = readVoxel(voxelData->voxels[sceneId], voxelIndex->index[sceneId], Vector3i((int)ROUND(point_local.x), (int)ROUND(point_local.y), (int)ROUND(point_local.z)), isFound, cache);
		if (!isFound) continue;

		hash_found = true;
		sum_sdf += (float)v.w_depth * (float)v.sdf;
		sum_weights += v.w_depth;
	}
	if (sum_weights == 0) return 1.0f;
	return TVoxel::SDF_valueToFloat(sum_sdf / (float)sum_weights);
}

template<class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_interpolated(const CONSTPTR(TVoxel) *voxelData,
	const CONSTPTR(TIndex) *voxelIndex, Vector3f point, THREADPTR(int) &vmIndex, THREADPTR(TCache) & cache, int & maxW)
{
	float res1, res2, v1, v2;
	Vector3f coeff; Vector3i pos; TO_INT_FLOOR3(pos, coeff, point);

	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache);
		v1 = v.sdf;
		maxW = v.w_depth;
	}
	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache);
		v2 = v.sdf;
		if (v.w_depth > maxW) maxW = v.w_depth;
	}
	res1 = (1.0f - coeff.x) * v1 + coeff.x * v2;

	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache);
		v1 = v.sdf;
		if (v.w_depth > maxW) maxW = v.w_depth;
	}
	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache);
		v2 = v.sdf;
		if (v.w_depth > maxW) maxW = v.w_depth;
	}
	res1 = (1.0f - coeff.y) * res1 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);

	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache);
		v1 = v.sdf;
		if (v.w_depth > maxW) maxW = v.w_depth;
	}
	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache);
		v2 = v.sdf;
		if (v.w_depth > maxW) maxW = v.w_depth;
	}
	res2 = (1.0f - coeff.x) * v1 + coeff.x * v2;

	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache);
		v1 = v.sdf;
		if (v.w_depth > maxW) maxW = v.w_depth;
	}
	{
		const TVoxel & v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache);
		v2 = v.sdf;
		if (v.w_depth > maxW) maxW = v.w_depth;
	}
	res2 = (1.0f - coeff.y) * res2 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);

	vmIndex = true;
	return TVoxel::SDF_valueToFloat((1.0f - coeff.z) * res1 + coeff.z * res2);
}

template<class TMultiVoxel, class TMultiIndex>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_interpolated(const TMultiVoxel *voxelData, const TMultiIndex *voxelIndex, const Vector3f & point, bool & hash_found, ITMLib::ITMMultiCache & _cache)
{
	typedef typename TMultiIndex::IndexType TIndex;

	float sum_sdf = 0.0f;
	int sum_weights = 0;
	hash_found = false;
	for (int sceneId = 0; sceneId < voxelIndex->numScenes; ++sceneId) {
		Vector3f point_local = voxelIndex->poses_vs[sceneId] * point;

		int isFound;
		int maxW;
		typename TIndex::IndexCache cache;
		float sdf = readFromSDF_float_interpolated(voxelData->voxels[sceneId], voxelIndex->index[sceneId], point_local, isFound, cache, maxW);
		if (!isFound) continue;
		hash_found = true;

		sum_sdf += (float)maxW * sdf;
		sum_weights += maxW;
	}
	if (sum_weights == 0) return 1.0f;
	return (sum_sdf / (float)sum_weights);
}

template<class TMultiVoxel, class TMultiIndex>
_CPU_AND_GPU_CODE_ inline Vector4f readFromSDF_color4u_interpolated(const TMultiVoxel *voxelData, const TMultiIndex *voxelIndex, const Vector3f & point, ITMLib::ITMMultiCache & _cache)
{
	typedef typename TMultiIndex::IndexData::IndexType TIndex;

	Vector4f accu = 0.0f;
	for (int sceneId = 0; sceneId < voxelIndex->numScenes; ++sceneId) {
		Vector3f point_local = voxelIndex->poses_vs[sceneId] * point;

		int maxW;
		typename TIndex::IndexCache cache;
		Vector4f val = readFromSDF_color4u_interpolated(voxelData->voxels[sceneId], voxelIndex->index[sceneId], point_local, cache, maxW);

		accu += (float)maxW * val;
	}
	if (accu.w < 0.001f) accu.w = 1.0f;
	return (accu / accu.w);
}

