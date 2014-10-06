// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMBlockProjectionEngine_CPU.h"
#include "../../DeviceAgnostic/ITMBlockProjectionEngine.h"

#include <vector>

using namespace ITMLib::Objects;
using namespace ITMLib::Engine;

template<class TVoxel>
void ITMBlockProjectionEngine_CPU<TVoxel,ITMVoxelBlockHash>::CreateExpectedDepths(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMImage<Vector2f> *minmaximg)
{
	Vector2i imgSize = minmaximg->noDims;
	Vector2f *minmaxData = minmaximg->GetData(false);

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

	const int *liveEntryIDs = scene->index.GetLiveEntryIDs();
	int noLiveEntries = scene->index.noLiveEntries;

	//go through list of visible 8x8x8 blocks
	for (int blockNo = 0; blockNo < noLiveEntries; ++blockNo) {
		const ITMHashEntry & blockData(scene->index.GetEntries()[liveEntryIDs[blockNo]]);

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
void ITMBlockProjectionEngine_CPU<TVoxel,TIndex>::CreateExpectedDepths(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMImage<Vector2f> *minmaximg)
{
	Vector2i imgSize = minmaximg->noDims;
	Vector2f *minmaxData = minmaximg->GetData(false);

	for (int y = 0; y < imgSize.y; ++y) {
		for (int x = 0; x < imgSize.x; ++x) {
			//TODO : this could be improved a bit...
			Vector2f & pixel = minmaxData[x + y*imgSize.x];
			pixel.x = 0.2f;
			pixel.y = 3.0f;
		}
	}
}

template class ITMBlockProjectionEngine_CPU<ITMVoxel,ITMVoxelIndex>;
