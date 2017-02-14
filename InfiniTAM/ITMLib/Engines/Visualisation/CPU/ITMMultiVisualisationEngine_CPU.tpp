// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMMultiVisualisationEngine_CPU.h"

#include "../../../Objects/RenderStates/ITMRenderStateMultiScene.h"
#include "../../../Objects/Scene/ITMMultiSceneAccess.h"

#include "../Shared/ITMVisualisationEngine_Shared.h"

using namespace ITMLib;

template<class TVoxel, class TIndex>
ITMRenderState* ITMMultiVisualisationEngine_CPU<TVoxel, TIndex>::CreateRenderState(const ITMScene<TVoxel, TIndex> *scene, const Vector2i & imgSize) const
{
	return new ITMRenderStateMultiScene<TVoxel, TIndex>(imgSize, scene->sceneParams->viewFrustum_min, scene->sceneParams->viewFrustum_max, MEMORYDEVICE_CPU);
}

template<class TVoxel, class TIndex>
void ITMMultiVisualisationEngine_CPU<TVoxel, TIndex>::PrepareRenderState(const ITMVoxelMapGraphManager<TVoxel, TIndex> & mapManager, ITMRenderState *_state)
{
	ITMRenderStateMultiScene<TVoxel, TIndex> *state = (ITMRenderStateMultiScene<TVoxel, TIndex>*)_state;

	state->PrepareLocalMaps(mapManager);
}

template<class TVoxel, class TIndex>
void ITMMultiVisualisationEngine_CPU<TVoxel, TIndex>::CreateExpectedDepths(const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *_renderState) const
{
	ITMRenderStateMultiScene<TVoxel, TIndex> *renderState = (ITMRenderStateMultiScene<TVoxel, TIndex>*)_renderState;

	// reset min max image
	Vector2i imgSize = renderState->renderingRangeImage->noDims;
	Vector2f *minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);

	for (int locId = 0; locId < imgSize.x*imgSize.y; ++locId) 
	{
		Vector2f & pixel = minmaxData[locId];
		pixel.x = FAR_AWAY;
		pixel.y = VERY_CLOSE;
	}

	// add the values from each local map
	for (int localMapId = 0; localMapId < renderState->indexData_host.numLocalMaps; ++localMapId) 
	{
		float voxelSize = renderState->sceneParams.voxelSize;
		const ITMHashEntry *hash_entries = renderState->indexData_host.index[localMapId];
		int noHashEntries = ITMVoxelBlockHash::noTotalEntries;

		std::vector<RenderingBlock> renderingBlocks(MAX_RENDERING_BLOCKS);
		int numRenderingBlocks = 0;

		Matrix4f localPose = pose->GetM() * renderState->indexData_host.posesInv[localMapId];
		for (int blockNo = 0; blockNo < noHashEntries; ++blockNo) {
			const ITMHashEntry & blockData(hash_entries[blockNo]);

			Vector2i upperLeft, lowerRight;
			Vector2f zRange;
			bool validProjection = false;
			if (blockData.ptr >= 0) {
				validProjection = ProjectSingleBlock(blockData.pos, localPose, intrinsics->projectionParamsSimple.all, imgSize, voxelSize, upperLeft, lowerRight, zRange);
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
		for (int blockNo = 0; blockNo < numRenderingBlocks; ++blockNo) 
		{
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
}

template<class TVoxel, class TIndex>
void ITMMultiVisualisationEngine_CPU<TVoxel, TIndex>::RenderImage(const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *_renderState, ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type) const
{
	ITMRenderStateMultiScene<TVoxel, TIndex> *renderState = (ITMRenderStateMultiScene<TVoxel, TIndex>*)_renderState;

	Vector2i imgSize = outputImage->noDims;
	Matrix4f invM = pose->GetInvM();

	// Generic Raycast
	float voxelSize = renderState->sceneParams.voxelSize;
	{
		Vector4f projParams = intrinsics->projectionParamsSimple.all;
		Vector4f invProjParams = InvertProjectionParams(projParams);

		const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
		float mu = renderState->sceneParams.mu;
		float oneOverVoxelSize = 1.0f / voxelSize;
		Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);

		typedef ITMMultiVoxel<TVoxel> VD;
		typedef ITMMultiIndex<TIndex> ID;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int locId = 0; locId < imgSize.x*imgSize.y; ++locId)
		{
			int y = locId / imgSize.x;
			int x = locId - y*imgSize.x;
			int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

			castRay<VD, ID, false>(pointsRay[locId], NULL, x, y, &renderState->voxelData_host, &renderState->indexData_host, invM, invProjParams, oneOverVoxelSize, mu, minmaximg[locId2]);
		}
	}

	Vector3f lightSource = -Vector3f(invM.getColumn(2));
	Vector4u *outRendering = outputImage->GetData(MEMORYDEVICE_CPU);
	Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);

	if ((type == IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME) &&
		(!TVoxel::hasColorInformation)) type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;

	switch (type) {
	case IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME:
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {
			Vector4f ptRay = pointsRay[locId];
			processPixelColour<ITMMultiVoxel<TVoxel>, ITMMultiIndex<TIndex> >(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, &(renderState->voxelData_host),
				&(renderState->indexData_host));
		}
		break;
	case IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL:
		if (intrinsics->FocalLengthSignsDiffer())
		{
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
			for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
			{
				int y = locId / imgSize.x, x = locId - y*imgSize.x;
				processPixelNormals_ImageNormals<true, true>(outRendering, pointsRay, imgSize, x, y, voxelSize, lightSource);
			}
		}
		else
		{
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
			for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
			{
				int y = locId / imgSize.x, x = locId - y*imgSize.x;
				processPixelNormals_ImageNormals<true, false>(outRendering, pointsRay, imgSize, x, y, voxelSize, lightSource);
			}
		}
		break;
	case IITMVisualisationEngine::RENDER_COLOUR_FROM_CONFIDENCE:
		if (intrinsics->FocalLengthSignsDiffer())
		{
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
			for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
			{
				int y = locId / imgSize.x, x = locId - y*imgSize.x;
				processPixelConfidence_ImageNormals<true, true>(outRendering, pointsRay, imgSize, x, y, voxelSize, lightSource);
			}
		}
		else
		{
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
			for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
			{
				int y = locId / imgSize.x, x = locId - y*imgSize.x;
				processPixelConfidence_ImageNormals<true, false>(outRendering, pointsRay, imgSize, x, y, voxelSize, lightSource);
			}
		}
		break;
	case IITMVisualisationEngine::RENDER_SHADED_GREYSCALE:
	default:
		if (intrinsics->FocalLengthSignsDiffer())
		{
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
			for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
			{
				int y = locId / imgSize.x, x = locId - y*imgSize.x;
				processPixelGrey_ImageNormals<true, true>(outRendering, pointsRay, imgSize, x, y, voxelSize, lightSource);
			}
		}
		else
		{
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
			for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
			{
				int y = locId / imgSize.x, x = locId - y*imgSize.x;
				processPixelGrey_ImageNormals<true, false>(outRendering, pointsRay, imgSize, x, y, voxelSize, lightSource);
			}
		}
		break;
	}
}

