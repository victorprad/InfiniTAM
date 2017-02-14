// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Utils/ITMMath.h"

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void combineVoxelDepthInformation(const CONSTPTR(TVoxel) & src, DEVICEPTR(TVoxel) & dst, int maxW)
{
	int newW = dst.w_depth;
	int oldW = src.w_depth;
	float newF = TVoxel::valueToFloat(dst.sdf);
	float oldF = TVoxel::valueToFloat(src.sdf);

	if (oldW == 0) return;

	newF = oldW * oldF + newW * newF;
	newW = oldW + newW;
	newF /= newW;
	newW = MIN(newW, maxW);

	dst.w_depth = newW;
	dst.sdf = TVoxel::floatToValue(newF);
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void combineVoxelColorInformation(const CONSTPTR(TVoxel) & src, DEVICEPTR(TVoxel) & dst, int maxW)
{
	int newW = dst.w_color;
	int oldW = src.w_color;
	Vector3f newC = dst.clr.toFloat() / 255.0f;
	Vector3f oldC = src.clr.toFloat() / 255.0f;

	if (oldW == 0) return;

	newC = oldC * (float)oldW + newC * (float)newW;
	newW = oldW + newW;
	newC /= (float)newW;
	newW = MIN(newW, maxW);

	dst.clr = TO_UCHAR3(newC * 255.0f);
	dst.w_color = (uchar)newW;
}


template<bool hasColor,class TVoxel> struct CombineVoxelInformation;

template<class TVoxel>
struct CombineVoxelInformation<false,TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(const CONSTPTR(TVoxel) & src, DEVICEPTR(TVoxel) & dst, int maxW)
	{
		combineVoxelDepthInformation(src, dst, maxW);
	}
};

template<class TVoxel>
struct CombineVoxelInformation<true,TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(const CONSTPTR(TVoxel) & src, DEVICEPTR(TVoxel) & dst, int maxW)
	{
		combineVoxelDepthInformation(src, dst, maxW);
		combineVoxelColorInformation(src, dst, maxW);
	}
};

