// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

struct RenderingBlock {
	Vector2s upperLeft;
	Vector2s lowerRight;
	Vector2f zRange;
};

#ifndef FAR_AWAY
#define FAR_AWAY 999999.9f
#endif

#ifndef VERY_CLOSE
#define VERY_CLOSE 0.05f
#endif

static const int renderingBlockSizeX = 16;
static const int renderingBlockSizeY = 16;

static const int MAX_RENDERING_BLOCKS = 65536;
//static const int MAX_RENDERING_BLOCKS = 16384;

_CPU_AND_GPU_CODE_ inline bool ProjectSingleBlock(const Vector3s & blockPos, const Matrix4f & pose, const Vector4f & intrinsics, const Vector2i & imgSize, float voxelSize, Vector2i & upperLeft, Vector2i & lowerRight, Vector2f & zRange)
{
	upperLeft = imgSize;
	lowerRight = Vector2i(-1, -1);
	zRange = Vector2f(FAR_AWAY, VERY_CLOSE);
	for (int corner = 0; corner < 8; ++corner) 
	{
		// project all 8 corners down to 2D image
		Vector3s tmp = blockPos;
		tmp.x += (corner & 1) ? 1 : 0;
		tmp.y += (corner & 2) ? 1 : 0;
		tmp.z += (corner & 4) ? 1 : 0;
		Vector4f pt3d(tmp.toFloat() * (float)SDF_BLOCK_SIZE * voxelSize, 1.0f);
		pt3d = pose * pt3d;
		if (pt3d.z < 1e-6) continue;

		Vector2f pt2d;
		pt2d.x = intrinsics.x * pt3d.x / pt3d.z + intrinsics.z;
		pt2d.y = intrinsics.y * pt3d.y / pt3d.z + intrinsics.w;

		// remember bounding box, zmin and zmax
		if (upperLeft.x > floorf(pt2d.x)) upperLeft.x = (int)floorf(pt2d.x);
		if (lowerRight.x < ceilf(pt2d.x)) lowerRight.x = (int)ceilf(pt2d.x);
		if (upperLeft.y > floorf(pt2d.y)) upperLeft.y = (int)floorf(pt2d.y);
		if (lowerRight.y < ceilf(pt2d.y)) lowerRight.y = (int)ceilf(pt2d.y);
		if (zRange.x > pt3d.z) zRange.x = pt3d.z;
		if (zRange.y < pt3d.z) zRange.y = pt3d.z;
	}

	// do some sanity checks and respect image bounds
	if (upperLeft.x < 0) upperLeft.x = 0;
	if (upperLeft.y < 0) upperLeft.y = 0;
	if (lowerRight.x >= imgSize.x) lowerRight.x = imgSize.x - 1;
	if (lowerRight.y >= imgSize.y) lowerRight.y = imgSize.y - 1;
	if (upperLeft.x > lowerRight.x) return false;
	if (upperLeft.y > lowerRight.y) return false;
	//if (zRange.y <= VERY_CLOSE) return false; never seems to happen
	if (zRange.x < VERY_CLOSE) zRange.x = VERY_CLOSE;
	if (zRange.y < VERY_CLOSE) return false;

	return true;
}

_CPU_AND_GPU_CODE_ inline void CreateRenderingBlocks(RenderingBlock *renderingBlockList, int offset, const Vector2i & upperLeft, const Vector2i & lowerRight, const Vector2f & zRange)
{
	// split bounding box into 16x16 pixel rendering blocks
	for (int by = 0; by < ceilf((float)(1 + lowerRight.y - upperLeft.y) / renderingBlockSizeY); ++by) {
		for (int bx = 0; bx < ceilf((float)(1 + lowerRight.x - upperLeft.x) / renderingBlockSizeX); ++bx) {
			if (offset >= MAX_RENDERING_BLOCKS) return;
			//for each rendering block: add it to the list
			RenderingBlock & b(renderingBlockList[offset++]);
			b.upperLeft.x = upperLeft.x + bx*renderingBlockSizeX;
			b.upperLeft.y = upperLeft.y + by*renderingBlockSizeY;
			b.lowerRight.x = upperLeft.x + (bx + 1)*renderingBlockSizeX - 1;
			b.lowerRight.y = upperLeft.y + (by + 1)*renderingBlockSizeY - 1;
			if (b.lowerRight.x>lowerRight.x) b.lowerRight.x = lowerRight.x;
			if (b.lowerRight.y>lowerRight.y) b.lowerRight.y = lowerRight.y;
			b.zRange = zRange;
		}
	}
}
