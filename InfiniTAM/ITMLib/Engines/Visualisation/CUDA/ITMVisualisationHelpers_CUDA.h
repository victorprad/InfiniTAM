// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/Tracking/ITMTrackingState.h"
#include "../../../Objects/Views/ITMView.h"

#include "../Shared/ITMVisualisationEngine_Shared.h"
#include "../../Reconstruction/Shared/ITMSceneReconstructionEngine_Shared.h"
#include "../../../Utils/ITMCUDAUtils.h"

namespace ITMLib
{
	// declaration of device functions

	__global__ void buildCompleteVisibleList_device(const ITMHashEntry *hashTable, /*ITMHashCacheState *cacheStates, bool useSwapping,*/ int noTotalEntries,
		int *visibleEntryIDs, int *noVisibleEntries, uchar *entriesVisibleType, Matrix4f M, Vector4f projParams, Vector2i imgSize, float voxelSize);

	__global__ void countVisibleBlocks_device(const int *visibleEntryIDs, int noVisibleEntries, const ITMHashEntry *hashTable, uint *noBlocks, int minBlockId, int maxBlockId);

	__global__ void projectAndSplitBlocks_device(const ITMHashEntry *hashEntries, const int *visibleEntryIDs, int noVisibleEntries,
		const Matrix4f pose_M, const Vector4f intrinsics, const Vector2i imgSize, float voxelSize, RenderingBlock *renderingBlocks,
		uint *noTotalBlocks);

	__global__ void checkProjectAndSplitBlocks_device(const ITMHashEntry *hashEntries, int noHashEntries,
		const Matrix4f pose_M, const Vector4f intrinsics, const Vector2i imgSize, float voxelSize, RenderingBlock *renderingBlocks,
		uint *noTotalBlocks);

	__global__ void fillBlocks_device(uint noTotalBlocks, const RenderingBlock *renderingBlocks,
		Vector2i imgSize, Vector2f *minmaxData);

	__global__ void findMissingPoints_device(int *fwdProjMissingPoints, uint *noMissingPoints, const Vector2f *minmaximg,
		Vector4f *forwardProjection, float *currentDepth, Vector2i imgSize);

	__global__ void forwardProject_device(Vector4f *forwardProjection, const Vector4f *pointsRay, Vector2i imgSize, Matrix4f M,
		Vector4f projParams, float voxelSize);

	template<class TVoxel, class TIndex, bool modifyVisibleEntries>
	__global__ void genericRaycast_device(Vector4f *out_ptsRay, uchar *entriesVisibleType, const TVoxel *voxelData,
		const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f invProjParams,
		float oneOverVoxelSize, const Vector2f *minmaximg, float mu)
	{
		int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

		if (x >= imgSize.x || y >= imgSize.y) return;

		int locId = x + y * imgSize.x;
		int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

		castRay<TVoxel, TIndex, modifyVisibleEntries>(out_ptsRay[locId], entriesVisibleType, x, y, voxelData, voxelIndex, invM, invProjParams, oneOverVoxelSize, mu, minmaximg[locId2]);
	}

	template<class TVoxel, class TIndex, bool modifyVisibleEntries>
	__global__ void genericRaycastMissingPoints_device(Vector4f *forwardProjection, uchar *entriesVisibleType, const TVoxel *voxelData,
		const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f invProjParams, float oneOverVoxelSize,
		int *fwdProjMissingPoints, int noMissingPoints, const Vector2f *minmaximg, float mu)
	{
		int pointId = threadIdx.x + blockIdx.x * blockDim.x;

		if (pointId >= noMissingPoints) return;

		int locId = fwdProjMissingPoints[pointId];
		int y = locId / imgSize.x, x = locId - y*imgSize.x;
		int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

		castRay<TVoxel, TIndex, modifyVisibleEntries>(forwardProjection[locId], entriesVisibleType, x, y, voxelData, voxelIndex, invM, invProjParams, oneOverVoxelSize, mu, minmaximg[locId2]);
	}

	template<bool flipNormals>
	__global__ void renderICP_device(Vector4f *pointsMap, Vector4f *normalsMap, const Vector4f *pointsRay,
		float voxelSize, Vector2i imgSize, Vector3f lightSource)
	{
		int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

		if (x >= imgSize.x || y >= imgSize.y) return;

		processPixelICP<true, flipNormals>(pointsMap, normalsMap, pointsRay, imgSize, x, y, voxelSize, lightSource);
	}

	template<bool flipNormals>
	__global__ void renderGrey_ImageNormals_device(Vector4u *outRendering, const Vector4f *pointsRay, float voxelSize, Vector2i imgSize, Vector3f lightSource)
	{
		int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

		if (x >= imgSize.x || y >= imgSize.y) return;

		processPixelGrey_ImageNormals<true, flipNormals>(outRendering, pointsRay, imgSize, x, y, voxelSize, lightSource);
	}

	template<bool flipNormals>
	__global__ void renderNormals_ImageNormals_device(Vector4u *outRendering, const Vector4f *ptsRay, Vector2i imgSize, float voxelSize, Vector3f lightSource)
	{
		int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

		if (x >= imgSize.x || y >= imgSize.y) return;

		processPixelNormals_ImageNormals<true, flipNormals>(outRendering, ptsRay, imgSize, x, y, voxelSize, lightSource);
	}

	template<bool flipNormals>
	__global__ void renderConfidence_ImageNormals_device(Vector4u *outRendering, const Vector4f *ptsRay, Vector2i imgSize, float voxelSize, Vector3f lightSource)
	{
		int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

		if (x >= imgSize.x || y >= imgSize.y) return;

		processPixelConfidence_ImageNormals<true, flipNormals>(outRendering, ptsRay, imgSize, x, y, voxelSize, lightSource);
	}

	template<class TVoxel, class TIndex>
	__global__ void renderGrey_device(Vector4u *outRendering, const Vector4f *ptsRay, const TVoxel *voxelData,
		const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Vector3f lightSource)
	{
		int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

		if (x >= imgSize.x || y >= imgSize.y) return;

		int locId = x + y * imgSize.x;

		Vector4f ptRay = ptsRay[locId];

		processPixelGrey<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource);
	}

	template<class TVoxel, class TIndex>
	__global__ void renderColourFromNormal_device(Vector4u *outRendering, const Vector4f *ptsRay, const TVoxel *voxelData,
		const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Vector3f lightSource)
	{
		int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

		if (x >= imgSize.x || y >= imgSize.y) return;

		int locId = x + y * imgSize.x;

		Vector4f ptRay = ptsRay[locId];

		processPixelNormal<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource);
	}

	template<class TVoxel, class TIndex>
	__global__ void renderColourFromConfidence_device(Vector4u *outRendering, const Vector4f *ptsRay, const TVoxel *voxelData,
		const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Vector3f lightSource)
	{
		int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

		if (x >= imgSize.x || y >= imgSize.y) return;

		int locId = x + y * imgSize.x;

		Vector4f ptRay = ptsRay[locId];

		processPixelConfidence<TVoxel, TIndex>(outRendering[locId], ptRay, ptRay.w > 0, voxelData, voxelIndex, lightSource);
	}

	template<class TVoxel, class TIndex>
	__global__ void renderPointCloud_device(/*Vector4u *outRendering, */Vector4f *locations, Vector4f *colours, uint *noTotalPoints,
		const Vector4f *ptsRay, const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, bool skipPoints,
		float voxelSize, Vector2i imgSize, Vector3f lightSource)
	{
		__shared__ bool shouldPrefix;
		shouldPrefix = false;
		__syncthreads();

		bool foundPoint = false; Vector3f point(0.0f);

		int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

		if (x < imgSize.x && y < imgSize.y)
		{
			int locId = x + y * imgSize.x;
			Vector3f outNormal; float angle; Vector4f pointRay;

			pointRay = ptsRay[locId];
			point = pointRay.toVector3();
			foundPoint = pointRay.w > 0;

			computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

			if (skipPoints && ((x % 2 == 0) || (y % 2 == 0))) foundPoint = false;

			if (foundPoint) shouldPrefix = true;
		}

		__syncthreads();

		if (shouldPrefix)
		{
			int offset = computePrefixSum_device<uint>(foundPoint, noTotalPoints, blockDim.x * blockDim.y, threadIdx.x + threadIdx.y * blockDim.x);

			if (offset != -1)
			{
				Vector4f tmp;
				tmp = VoxelColorReader<TVoxel::hasColorInformation, TVoxel, TIndex>::interpolate(voxelData, voxelIndex, point);
				if (tmp.w > 0.0f) { tmp.x /= tmp.w; tmp.y /= tmp.w; tmp.z /= tmp.w; tmp.w = 1.0f; }
				colours[offset] = tmp;

				Vector4f pt_ray_out;
				pt_ray_out.x = point.x * voxelSize; pt_ray_out.y = point.y * voxelSize;
				pt_ray_out.z = point.z * voxelSize; pt_ray_out.w = 1.0f;
				locations[offset] = pt_ray_out;
			}
		}
	}

	template<class TVoxel, class TIndex>
	__global__ void renderColour_device(Vector4u *outRendering, const Vector4f *ptsRay, const TVoxel *voxelData,
		const typename TIndex::IndexData *voxelIndex, Vector2i imgSize)
	{
		int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

		if (x >= imgSize.x || y >= imgSize.y) return;

		int locId = x + y * imgSize.x;

		Vector4f ptRay = ptsRay[locId];

		processPixelColour<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex);
	}
}
