// Copyright 2016 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Objects/Scene/ITMRepresentationAccess.h"

#define MAX_NUM_SCENES 32

namespace ITMLib {
	struct ITMMultiCache {};

	template<class TIndex>
	class ITMMultiIndex
	{
	public:
		typedef TIndex IndexType;
		typedef ITMMultiCache IndexCache;

		struct IndexData
		{
			int numScenes;
			typedef TIndex IndexType;
			typename TIndex::IndexData *index[MAX_NUM_SCENES];
			Matrix4f poses_vs[MAX_NUM_SCENES];
			Matrix4f posesInv[MAX_NUM_SCENES];
		};
	};

	template<class TVoxel>
	class ITMMultiVoxel
	{
	public:
		typedef TVoxel VoxelType;
		TVoxel* voxels[MAX_NUM_SCENES];

		static const CONSTPTR(bool) hasColorInformation = TVoxel::hasColorInformation;
	};
}

template<class TMultiVoxel, class TMultiIndex>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_uninterpolated(const TMultiVoxel *voxelData, const TMultiIndex *voxelIndex, const Vector3f & point, int & vmIndex, ITMLib::ITMMultiCache & _cache)
{
	typedef typename TMultiVoxel::VoxelType TVoxel;
	typedef typename TMultiIndex::IndexType TIndex;

	float sum_sdf = 0.0f;
	int sum_weights = 0;
	vmIndex = false;
	for (int sceneId = 0; sceneId < voxelIndex->numScenes; ++sceneId) {
		Vector3f point_local = voxelIndex->poses_vs[sceneId] * point;

		int vmIndex_tmp;
		typename TIndex::IndexCache cache;
		const TVoxel & v = readVoxel(voxelData->voxels[sceneId], voxelIndex->index[sceneId], Vector3i((int)ROUND(point_local.x), (int)ROUND(point_local.y), (int)ROUND(point_local.z)), vmIndex_tmp, cache);
		if (!vmIndex_tmp) continue;

		vmIndex = true;
		sum_sdf += (float)v.w_depth * (float)v.sdf;
		sum_weights += v.w_depth;
	}
	if (sum_weights == 0) return 1.0f;
	return TVoxel::valueToFloat(sum_sdf / (float)sum_weights);
}

template<class TMultiVoxel, class TMultiIndex>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_interpolated(const TMultiVoxel *voxelData, const TMultiIndex *voxelIndex, const Vector3f & point, int & vmIndex, ITMLib::ITMMultiCache & _cache)
{
	typedef typename TMultiIndex::IndexType TIndex;

	float sum_sdf = 0.0f;
	int sum_weights = 0;
	vmIndex = false;
	for (int sceneId = 0; sceneId < voxelIndex->numScenes; ++sceneId) {
		Vector3f point_local = voxelIndex->poses_vs[sceneId] * point;

		int vmIndex_tmp;
		int maxW;
		typename TIndex::IndexCache cache;
		float sdf = readFromSDF_float_interpolated(voxelData->voxels[sceneId], voxelIndex->index[sceneId], point_local, vmIndex_tmp, cache, maxW);
		if (!vmIndex_tmp) continue;
		vmIndex = true;

		sum_sdf += (float)maxW * sdf;
		sum_weights += maxW;
	}
	if (sum_weights == 0) return 1.0f;
	return (sum_sdf / (float)sum_weights);
}

template<class TMultiVoxel, class TMultiIndex>
_CPU_AND_GPU_CODE_ inline Vector4f readFromSDF_color4u_interpolated(const TMultiVoxel *voxelData, const TMultiIndex *voxelIndex, const Vector3f & point, ITMLib::ITMMultiCache & _cache)
{
	typedef typename TMultiIndex::IndexType TIndex;

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

template<class TMultiVoxel, class TMultiIndex>
_CPU_AND_GPU_CODE_ inline float readWithConfidenceFromSDF_float_interpolated(THREADPTR(float) &confidence, const CONSTPTR(TMultiVoxel) *voxelData,
	const CONSTPTR(TMultiIndex) *voxelIndex, Vector3f point, THREADPTR(int) &vmIndex, ITMLib::ITMMultiCache & _cache)
{
	typedef typename TMultiIndex::IndexType TIndex;

	float sum_sdf = 0.0f;
	float sum_confidence = 0.0f;
	vmIndex = false;
	for (int sceneId = 0; sceneId < voxelIndex->numScenes; ++sceneId) {
		Vector3f point_local = voxelIndex->poses_vs[sceneId] * point;

		int vmIndex_tmp;
		typename TIndex::IndexCache cache;
		float conf;
		float sdf = readWithConfidenceFromSDF_float_interpolated(conf, voxelData->voxels[sceneId], voxelIndex->index[sceneId], point_local, vmIndex_tmp, cache);
		if (!vmIndex_tmp) continue;
		vmIndex = true;

		sum_sdf += (float)conf * sdf;
		sum_confidence += conf;
	}
	if (sum_confidence == 0.0f) return 1.0f;
	return (sum_sdf / (float)sum_confidence);
}