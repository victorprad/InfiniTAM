// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMVisualisationEngine_CPU.h"
#include "../../DeviceAgnostic/ITMRepresentationAccess.h"
#include "../../DeviceAgnostic/ITMVisualisationEngine.h"
#include "../../DeviceAgnostic/ITMSceneReconstructionEngine.h"
#include "../../../Objects/ITMRenderState_VH.h"

#include <vector>

using namespace ITMLib::Engine;

template<class TVoxel, class TIndex>
static int RenderPointCloud(Vector4u *outRendering, Vector4f *locations, Vector4f *colours, const Vector4f *ptsRay, 
	const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, bool skipPoints, float voxelSize, 
	Vector2i imgSize, Vector3f lightSource);

template<class TVoxel, class TIndex>
ITMRenderState* ITMVisualisationEngine_CPU<TVoxel, TIndex>::CreateRenderState(const ITMScene<TVoxel, TIndex> *scene, const Vector2i & imgSize)
{
	return new ITMRenderState(imgSize, scene->sceneParams->viewFrustum_min, scene->sceneParams->viewFrustum_max, MEMORYDEVICE_CPU);
}

template<class TVoxel>
ITMRenderState* ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash>::CreateRenderState(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const Vector2i & imgSize)
{
	return new ITMRenderState_VH(ITMHashTable::noTotalEntries, imgSize, scene->sceneParams->viewFrustum_min, scene->sceneParams->viewFrustum_max, MEMORYDEVICE_CPU);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel,TIndex>::FindVisibleBlocks(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, 
	const ITMIntrinsics *intrinsics, ITMRenderState *renderState)
{
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHash>::FindVisibleBlocks(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMPose *pose, 
	const ITMIntrinsics *intrinsics, ITMRenderState *renderState)
{
	const ITMHashEntry *hashTable = scene->index.GetEntries();
	int noTotalEntries = scene->index.noVoxelBlocks;
	float voxelSize = scene->sceneParams->voxelSize;
	Vector2i imgSize = renderState->renderingRangeImage->noDims;

	Matrix4f M = pose->M;
	Vector4f projParams = intrinsics->projectionParamsSimple.all;

	ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState;

	int noVisibleEntries = 0;
	int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();

	//build visible list
	for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
	{
		unsigned char hashVisibleType = 0;// = entriesVisibleType[targetIdx];
		const ITMHashEntry &hashEntry = hashTable[targetIdx];

		if (hashEntry.ptr >= 0)
		{
			bool isVisible, isVisibleEnlarged;
			checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M, projParams, voxelSize, imgSize);
			hashVisibleType = isVisible;
		}

		if (hashVisibleType > 0)
		{
			visibleEntryIDs[noVisibleEntries] = targetIdx;
			noVisibleEntries++;
		}
	}

	renderState_vh->noVisibleEntries = noVisibleEntries;
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel,TIndex>::CreateExpectedDepths(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, 
	const ITMIntrinsics *intrinsics, ITMRenderState *renderState)
{
	Vector2i imgSize = renderState->renderingRangeImage->noDims;
	Vector2f *minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < imgSize.y; ++y) {
		for (int x = 0; x < imgSize.x; ++x) {
			//TODO : this could be improved a bit...
			Vector2f & pixel = minmaxData[x + y*imgSize.x];
			pixel.x = 0.2f;
			pixel.y = 3.0f;
		}
	}
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHash>::CreateExpectedDepths(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, 
	const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState)
{
	Vector2i imgSize = renderState->renderingRangeImage->noDims;
	Vector2f *minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < imgSize.y; ++y) {
		for (int x = 0; x < imgSize.x; ++x) {
			Vector2f & pixel = minmaxData[x + y*imgSize.x];
			pixel.x = FAR_AWAY;
			pixel.y = VERY_CLOSE;
		}
	}

	float voxelSize = scene->sceneParams->voxelSize;

	std::vector<RenderingBlock> renderingBlocks(MAX_RENDERING_BLOCKS);
	int numRenderingBlocks = 0;

	ITMRenderState_VH* renderState_vh = (ITMRenderState_VH*)renderState;

	const int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();
	int noVisibleEntries = renderState_vh->noVisibleEntries;

	//go through list of visible 8x8x8 blocks
	for (int blockNo = 0; blockNo < noVisibleEntries; ++blockNo) {
		const ITMHashEntry & blockData(scene->index.GetEntries()[visibleEntryIDs[blockNo]]);

		Vector2i upperLeft, lowerRight;
		Vector2f zRange;
		bool validProjection = false;
		if (blockData.ptr>=0) {
			validProjection = ProjectSingleBlock(blockData.pos, pose->M, intrinsics->projectionParamsSimple.all, imgSize, voxelSize, upperLeft, lowerRight, zRange);
		}
		if (!validProjection) continue;

		Vector2i requiredRenderingBlocks((int)ceilf((float)(lowerRight.x - upperLeft.x + 1) / (float)renderingBlockSizeX), 
			(int)ceilf((float)(lowerRight.y - upperLeft.y + 1) / (float)renderingBlockSizeY));
		int requiredNumBlocks = requiredRenderingBlocks.x * requiredRenderingBlocks.y;

		if (numRenderingBlocks + requiredNumBlocks >= MAX_RENDERING_BLOCKS) continue;
		int offset = numRenderingBlocks;
		numRenderingBlocks += requiredNumBlocks;

		CreateRenderingBlocks(&(renderingBlocks[0]), offset, upperLeft, lowerRight, zRange);
	}

	// go through rendering blocks
	for (int blockNo = 0; blockNo < numRenderingBlocks; ++blockNo) {
		// fill minmaxData
		const RenderingBlock & b(renderingBlocks[blockNo]);

		for (int y = b.upperLeft.y; y <= b.lowerRight.y; ++y) {
			for (int x = b.upperLeft.x; x <= b.lowerRight.x; ++x) {
				Vector2f & pixel(minmaxData[x + y*imgSize.x]);
				if (pixel.x > b.zRange.x) pixel.x = b.zRange.x;
				if (pixel.y < b.zRange.y) pixel.y = b.zRange.y;
			}
		}
	}
}

template<class TVoxel, class TIndex>
static void RenderImage_common(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, 
	const ITMRenderState *renderState, ITMUChar4Image *outputImage, bool useColour)
{
	const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
	const typename TIndex::IndexData *voxelIndex = scene->index.getIndexData();

	Vector2i imgSize = outputImage->noDims;
	float voxelSize = scene->sceneParams->voxelSize;
	float oneOverVoxelSize = 1.0f / voxelSize;

	Matrix4f invM = pose->invM;
	Vector4f projParams = intrinsics->projectionParamsSimple.all;
	projParams.x = 1.0f / projParams.x;
	projParams.y = 1.0f / projParams.y;

	const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
	Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
	{
		int locId = x + y * imgSize.x;
		int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

		castRay<TVoxel, TIndex>(pointsRay[locId], x, y, voxelData, voxelIndex, invM, projParams, oneOverVoxelSize, scene->sceneParams->mu, minmaximg[locId2]);
	}

	Vector3f lightSource = ComputeLightSource(invM);
	Vector4u *outRendering = outputImage->GetData(MEMORYDEVICE_CPU);

	if (useColour && TVoxel::hasColorInformation)
	{
		for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
		{
			Vector4f ptRay = pointsRay[locId];
			processPixelColour<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource);
		}
	}
	else 
	{
		for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
		{
			Vector4f ptRay = pointsRay[locId];
			processPixelGrey<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource);
		}
	}
}

template<class TVoxel, class TIndex>
static void CreatePointCloud_common(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, 
	ITMRenderState *renderState, bool skipPoints)
{
	const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
	const typename TIndex::IndexData *voxelIndex = scene->index.getIndexData();

	Vector2i imgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;
	float oneOverVoxelSize = 1.0f / voxelSize;

	Matrix4f invM = trackingState->pose_d->invM * view->calib->trafo_rgb_to_depth.calib;
	Vector4f projParams = view->calib->intrinsics_rgb.projectionParamsSimple.all;
	projParams.x = 1.0f / projParams.x;
	projParams.y = 1.0f / projParams.y;

	const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
	Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
	{
		int locId = x + y * imgSize.x;
		int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

		castRay<TVoxel, TIndex>(pointsRay[locId], x, y, voxelData, voxelIndex, invM, projParams, oneOverVoxelSize, scene->sceneParams->mu, minmaximg[locId2]);
	}

	Vector4f *colours = trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CPU);
	Vector3f lightSource = ComputeLightSource(invM);
	Vector4f *locations = trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CPU);
	Vector4u *outRendering = renderState->raycastImage->GetData(MEMORYDEVICE_CPU);

	trackingState->pointCloud->noTotalPoints = RenderPointCloud<TVoxel, TIndex>(outRendering, locations, colours, pointsRay, 
		scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), skipPoints, voxelSize, imgSize, lightSource);
}

template<class TVoxel, class TIndex>
static void CreateICPMaps_common(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState)
{
	const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
	const typename TIndex::IndexData *voxelIndex = scene->index.getIndexData();

	Vector2i imgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;
	float oneOverVoxelSize = 1.0f / voxelSize;

	Matrix4f invM = trackingState->pose_d->invM;
	Vector4f projParams = view->calib->intrinsics_d.projectionParamsSimple.all;
	projParams.x = 1.0f / projParams.x;
	projParams.y = 1.0f / projParams.y;

	const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
	Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
	{
		int locId = x + y * imgSize.x;
		int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

		castRay<TVoxel, TIndex>(pointsRay[locId], x, y, voxelData, voxelIndex, invM, projParams, oneOverVoxelSize, scene->sceneParams->mu, minmaximg[locId2]);
	}

	Vector3f lightSource = ComputeLightSource(invM);
	Vector4f *normalsMap = trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CPU);
	Vector4u *outRendering = renderState->raycastImage->GetData(MEMORYDEVICE_CPU);
	Vector4f *pointsMap = trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CPU);

	for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
	{
		Vector4f ptRay = pointsRay[locId];
		processPixelICP<TVoxel, TIndex>(outRendering[locId], pointsMap[locId], normalsMap[locId], ptRay.toVector3(), ptRay.w > 0, voxelData,
			voxelIndex, voxelSize, lightSource);
	}
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel,TIndex>::RenderImage(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, 
	const ITMRenderState *renderState, ITMUChar4Image *outputImage, bool useColour)
{
	RenderImage_common(scene, pose, intrinsics, renderState, outputImage, useColour);
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHash>::RenderImage(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMPose *pose, 
	const ITMIntrinsics *intrinsics, const ITMRenderState *renderState, ITMUChar4Image *outputImage, bool useColour)
{
	RenderImage_common(scene, pose, intrinsics, renderState, outputImage, useColour);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel,TIndex>::FindSurface(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
	const ITMRenderState *renderState)
{
	// TODO
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHash>::FindSurface(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMPose *pose,
	const ITMIntrinsics *intrinsics, const ITMRenderState *renderState)
{
	// TODO
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel,TIndex>::CreatePointCloud(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, 
	ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints)
{
	CreatePointCloud_common(scene, view, trackingState, renderState, skipPoints);
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHash>::CreatePointCloud(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, 
	ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints)
{
	CreatePointCloud_common(scene, view, trackingState, renderState, skipPoints);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel,TIndex>::CreateICPMaps(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, 
	ITMTrackingState *trackingState, ITMRenderState *renderState)
{
	CreateICPMaps_common(scene, view, trackingState, renderState);
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHash>::CreateICPMaps(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, 
	ITMTrackingState *trackingState, ITMRenderState *renderState)
{
	CreateICPMaps_common(scene, view, trackingState, renderState);
}

template<class TVoxel, class TIndex>
static int RenderPointCloud(Vector4u *outRendering, Vector4f *locations, Vector4f *colours, const Vector4f *ptsRay, 
	const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, bool skipPoints, float voxelSize, 
	Vector2i imgSize, Vector3f lightSource)
{
	int noTotalPoints = 0;

	for (int y = 0, locId = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++, locId++)
	{
		Vector3f outNormal; float angle; 
		Vector4f pointRay = ptsRay[locId];
		Vector3f point = pointRay.toVector3();
		bool foundPoint = pointRay.w > 0;

		computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

		if (foundPoint) drawPixelGrey(outRendering[locId], angle);
		else outRendering[locId] = Vector4u((uchar)0);

		if (skipPoints && ((x % 2 == 0) || (y % 2 == 0))) foundPoint = false;

		if (foundPoint)
		{
			Vector4f tmp;
			tmp = VoxelColorReader<TVoxel::hasColorInformation, TVoxel, TIndex>::interpolate(voxelData, voxelIndex, point);
			if (tmp.w > 0.0f) { tmp.x /= tmp.w; tmp.y /= tmp.w; tmp.z /= tmp.w; tmp.w = 1.0f; }
			colours[noTotalPoints] = tmp;

			Vector4f pt_ray_out;
			pt_ray_out.x = point.x * voxelSize; pt_ray_out.y = point.y * voxelSize;
			pt_ray_out.z = point.z * voxelSize; pt_ray_out.w = 1.0f;
			locations[noTotalPoints] = pt_ray_out;

			noTotalPoints++;
		}
	}

	return noTotalPoints;
}

template class ITMLib::Engine::ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>;
