// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

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
ITMRenderState* ITMVisualisationEngine_CPU<TVoxel, TIndex>::CreateRenderState(const Vector2i & imgSize) const
{
	return new ITMRenderState(
		imgSize, this->scene->sceneParams->viewFrustum_min, this->scene->sceneParams->viewFrustum_max, MEMORYDEVICE_CPU
	);
}

template<class TVoxel>
ITMRenderState_VH* ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash>::CreateRenderState(const Vector2i & imgSize) const
{
	return new ITMRenderState_VH(
		ITMVoxelBlockHash::noTotalEntries, imgSize, this->scene->sceneParams->viewFrustum_min, this->scene->sceneParams->viewFrustum_max, MEMORYDEVICE_CPU
	);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel, TIndex>::FindVisibleBlocks(const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const
{
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHash>::FindVisibleBlocks(const ITMPose *pose, const ITMIntrinsics *intrinsics, 
	ITMRenderState *renderState) const
{
	const ITMHashEntry *hashTable = this->scene->index.GetEntries();
	int noTotalEntries = this->scene->index.noTotalEntries;
	float voxelSize = this->scene->sceneParams->voxelSize;
	Vector2i imgSize = renderState->renderingRangeImage->noDims;

	Matrix4f M = pose->GetM();
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
void ITMVisualisationEngine_CPU<TVoxel, TIndex>::CreateExpectedDepths(const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const
{
	Vector2i imgSize = renderState->renderingRangeImage->noDims;
	Vector2f *minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);

	for (int locId = 0; locId < imgSize.x*imgSize.y; ++locId) {
		//TODO : this could be improved a bit...
		Vector2f & pixel = minmaxData[locId];
		pixel.x = 0.2f;
		pixel.y = 3.0f;
	}
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHash>::CreateExpectedDepths(const ITMPose *pose, const ITMIntrinsics *intrinsics, 
	ITMRenderState *renderState) const
{
	Vector2i imgSize = renderState->renderingRangeImage->noDims;
	Vector2f *minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);

	for (int locId = 0; locId < imgSize.x*imgSize.y; ++locId) {
		Vector2f & pixel = minmaxData[locId];
		pixel.x = FAR_AWAY;
		pixel.y = VERY_CLOSE;
	}

	float voxelSize = this->scene->sceneParams->voxelSize;

	std::vector<RenderingBlock> renderingBlocks(MAX_RENDERING_BLOCKS);
	int numRenderingBlocks = 0;

	ITMRenderState_VH* renderState_vh = (ITMRenderState_VH*)renderState;

	const int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();
	int noVisibleEntries = renderState_vh->noVisibleEntries;

	//go through list of visible 8x8x8 blocks
	for (int blockNo = 0; blockNo < noVisibleEntries; ++blockNo) {
		const ITMHashEntry & blockData(this->scene->index.GetEntries()[visibleEntryIDs[blockNo]]);

		Vector2i upperLeft, lowerRight;
		Vector2f zRange;
		bool validProjection = false;
		if (blockData.ptr>=0) {
			validProjection = ProjectSingleBlock(blockData.pos, pose->GetM(), intrinsics->projectionParamsSimple.all, imgSize, voxelSize, upperLeft, lowerRight, zRange);
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
static void GenericRaycast(const ITMScene<TVoxel,TIndex> *scene, const Vector2i& imgSize, const Matrix4f& invM, Vector4f projParams, const ITMRenderState *renderState)
{
	projParams.x = 1.0f / projParams.x;
	projParams.y = 1.0f / projParams.y;

	const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
	float mu = scene->sceneParams->mu;
	float oneOverVoxelSize = 1.0f / scene->sceneParams->voxelSize;
	Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
	const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
	const typename TIndex::IndexData *voxelIndex = scene->index.getIndexData();

#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (int locId = 0; locId < imgSize.x*imgSize.y; ++locId)
	{
		int y = locId/imgSize.x;
		int x = locId - y*imgSize.x;
		int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

		castRay<TVoxel, TIndex>(
			pointsRay[locId],
			x, y,
			voxelData,
			voxelIndex,
			invM,
			projParams,
			oneOverVoxelSize,
			mu,
			minmaximg[locId2]
		);
	}
}

template<class TVoxel, class TIndex>
static void RenderImage_common(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, 
	const ITMRenderState *renderState, ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type)
{
	Vector2i imgSize = outputImage->noDims;
	Matrix4f invM = pose->GetInvM();

	GenericRaycast(scene, imgSize, invM, intrinsics->projectionParamsSimple.all, renderState);

	Vector3f lightSource = -Vector3f(invM.getColumn(2));
	Vector4u *outRendering = outputImage->GetData(MEMORYDEVICE_CPU);
	Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
	const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
	const typename TIndex::IndexData *voxelIndex = scene->index.getIndexData();

	if ((type == IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME)&&
	    (!TVoxel::hasColorInformation)) type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;

	switch (type) {
	case IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME:
#ifdef WITH_OPENMP
		#pragma omp parallel for
#endif
		for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
		{
			Vector4f ptRay = pointsRay[locId];
			processPixelColour<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource);
		}
		break;
	case IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL:
#ifdef WITH_OPENMP
		#pragma omp parallel for
#endif
		for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
		{
			Vector4f ptRay = pointsRay[locId];
			processPixelNormal<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource);
		}
		break;
	case IITMVisualisationEngine::RENDER_SHADED_GREYSCALE:
	default:
#ifdef WITH_OPENMP
		#pragma omp parallel for
#endif
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
	Vector2i imgSize = renderState->raycastResult->noDims;
	Matrix4f invM = trackingState->pose_d->GetInvM() * view->calib->trafo_rgb_to_depth.calib;

	GenericRaycast(scene, imgSize, invM, view->calib->intrinsics_rgb.projectionParamsSimple.all, renderState);
	trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);

	trackingState->pointCloud->noTotalPoints = RenderPointCloud<TVoxel, TIndex>(
		renderState->raycastImage->GetData(MEMORYDEVICE_CPU),
		trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CPU),
		trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CPU),
		renderState->raycastResult->GetData(MEMORYDEVICE_CPU),
		scene->localVBA.GetVoxelBlocks(),
		scene->index.getIndexData(),
		skipPoints,
		scene->sceneParams->voxelSize,
		imgSize,
		-Vector3f(invM.getColumn(2))
	);
}

template<class TVoxel, class TIndex>
static void CreateICPMaps_common(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState)
{
	Vector2i imgSize = renderState->raycastResult->noDims;
	Matrix4f invM = trackingState->pose_d->GetInvM();

	GenericRaycast(scene, imgSize, invM, view->calib->intrinsics_d.projectionParamsSimple.all, renderState);
	trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);

	Vector3f lightSource = -Vector3f(invM.getColumn(2));
	Vector4f *normalsMap = trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CPU);
	Vector4u *outRendering = renderState->raycastImage->GetData(MEMORYDEVICE_CPU);
	Vector4f *pointsMap = trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CPU);
	Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
	float voxelSize = scene->sceneParams->voxelSize;

#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		processPixelICP<true>(outRendering, pointsMap, normalsMap, pointsRay, imgSize, x, y, voxelSize, lightSource);
}

template<class TVoxel, class TIndex>
static void ForwardRender_common(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState)
{
	Vector2i imgSize = renderState->raycastResult->noDims;
	Matrix4f M = trackingState->pose_d->GetM();
	Matrix4f invM = trackingState->pose_d->GetInvM();
	Vector4f projParams = view->calib->intrinsics_d.projectionParamsSimple.all;
	Vector4f invProjParams = view->calib->intrinsics_d.projectionParamsSimple.all;
	invProjParams.x = 1.0f / invProjParams.x;
	invProjParams.y = 1.0f / invProjParams.y;

	Vector3f lightSource = -Vector3f(invM.getColumn(2));
	const Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
	Vector4f *forwardProjection = renderState->forwardProjection->GetData(MEMORYDEVICE_CPU);
	float *currentDepth = view->depth->GetData(MEMORYDEVICE_CPU);
	int *fwdProjMissingPoints = renderState->fwdProjMissingPoints->GetData(MEMORYDEVICE_CPU);
	Vector4u *outRendering = renderState->raycastImage->GetData(MEMORYDEVICE_CPU);
	const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
	float voxelSize = scene->sceneParams->voxelSize;
	const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
	const typename TIndex::IndexData *voxelIndex = scene->index.getIndexData();

	renderState->forwardProjection->Clear();

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
	{
		int locId = x + y * imgSize.x;
		Vector4f pixel = pointsRay[locId];

		int locId_new = forwardProjectPixel(pixel * voxelSize, M, projParams, imgSize);
		if (locId_new >= 0) forwardProjection[locId_new] = pixel;
	}

	int noMissingPoints = 0;
	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
	{
		int locId = x + y * imgSize.x;
		int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

		Vector4f fwdPoint = forwardProjection[locId];
		Vector2f minmaxval = minmaximg[locId2];
		float depth = currentDepth[locId];

		if ((fwdPoint.w <= 0) && ((fwdPoint.x == 0 && fwdPoint.y == 0 && fwdPoint.z == 0) || (depth >= 0)) && (minmaxval.x < minmaxval.y))
		//if ((fwdPoint.w <= 0) && (minmaxval.x < minmaxval.y))
		{
			fwdProjMissingPoints[noMissingPoints] = locId;
			noMissingPoints++;
		}
	}

	renderState->noFwdProjMissingPoints = noMissingPoints;
    
	for (int pointId = 0; pointId < noMissingPoints; pointId++)
	{
		int locId = fwdProjMissingPoints[pointId];
		int y = locId / imgSize.x, x = locId - y*imgSize.x;
		int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

		castRay<TVoxel, TIndex>(forwardProjection[locId], x, y, voxelData, voxelIndex, invM, invProjParams,
			1.0f / scene->sceneParams->voxelSize, scene->sceneParams->mu, minmaximg[locId2]);
	}

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		processPixelForwardRender<true>(outRendering, forwardProjection, imgSize, x, y, voxelSize, lightSource);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel,TIndex>::RenderImage(const ITMPose *pose, const ITMIntrinsics *intrinsics, 
	const ITMRenderState *renderState, ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type) const
{
	RenderImage_common(this->scene, pose, intrinsics, renderState, outputImage, type);
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHash>::RenderImage(const ITMPose *pose,  const ITMIntrinsics *intrinsics, 
	const ITMRenderState *renderState, ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type) const
{
	RenderImage_common(this->scene, pose, intrinsics, renderState, outputImage, type);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel, TIndex>::FindSurface(const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const
{
	GenericRaycast(this->scene, renderState->raycastResult->noDims, pose->GetInvM(), intrinsics->projectionParamsSimple.all, renderState);
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHash>::FindSurface(const ITMPose *pose, const ITMIntrinsics *intrinsics, 
	const ITMRenderState *renderState) const
{
	GenericRaycast(this->scene, renderState->raycastResult->noDims, pose->GetInvM(), intrinsics->projectionParamsSimple.all, renderState);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel,TIndex>::CreatePointCloud(const ITMView *view, ITMTrackingState *trackingState, 
	ITMRenderState *renderState, bool skipPoints) const
{ 
	CreatePointCloud_common(this->scene, view, trackingState, renderState, skipPoints);
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHash>::CreatePointCloud(const ITMView *view, ITMTrackingState *trackingState, 
	ITMRenderState *renderState, bool skipPoints) const
{
	CreatePointCloud_common(this->scene, view, trackingState, renderState, skipPoints);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel,TIndex>::CreateICPMaps(const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const
{
	CreateICPMaps_common(this->scene, view, trackingState, renderState);
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHash>::CreateICPMaps(const ITMView *view, ITMTrackingState *trackingState, 
	ITMRenderState *renderState) const
{
	CreateICPMaps_common(this->scene, view, trackingState, renderState);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel, TIndex>::ForwardRender(const ITMView *view, ITMTrackingState *trackingState, 
	ITMRenderState *renderState) const
{
	ForwardRender_common(this->scene, view, trackingState, renderState);
}

template<class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash>::ForwardRender(const ITMView *view, ITMTrackingState *trackingState,
	ITMRenderState *renderState) const
{
	ForwardRender_common(this->scene, view, trackingState, renderState);
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
