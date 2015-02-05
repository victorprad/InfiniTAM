// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMLibDefines.h"

_CPU_AND_GPU_CODE_ inline void convertDisparityToDepth(DEVICEPTR(float) *d_out, int x, int y, const CONSTANT(short) *d_in,
	Vector2f disparityCalibParams, float fx_depth, Vector2i imgSize)
{
	int locId = x + y * imgSize.x;

	short disparity = d_in[locId];
	float disparity_tmp = disparityCalibParams.x - (float)(disparity);
	float depth;

	if (disparity_tmp == 0) depth = 0.0;
	else depth = 8.0f * disparityCalibParams.y * fx_depth / disparity_tmp;

	d_out[locId] = (depth > 0) ? depth : -1.0f;
}

_CPU_AND_GPU_CODE_ inline void convertDepthMMToFloat(DEVICEPTR(float) *d_out, int x, int y, const CONSTANT(short) *d_in, Vector2i imgSize)
{
	int locId = x + y * imgSize.x;

	short depth_in = d_in[locId];
	d_out[locId] = ((depth_in <= 0)||(depth_in > 32000)) ? -1.0f : (float)depth_in / 1000.0f;
}