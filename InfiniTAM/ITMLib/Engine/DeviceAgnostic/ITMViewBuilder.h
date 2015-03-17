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

static _CPU_AND_GPU_CONSTANT_ float conv[25] = { 0.4493f, 0.6065f, 0.6703f, 0.6065f, 0.4493f, 0.6065f, 0.8187f, 0.9048f, 0.8187f, 0.6065f, 0.6703f, 0.9048f, 
1.0000f, 0.9048f, 0.6703f, 0.6065f, 0.8187f, 0.9048f, 0.8187f, 0.6065f, 0.4493f, 0.6065f, 0.6703f, 0.6065f, 0.4493f };

_CPU_AND_GPU_CODE_ inline void filterDepth(DEVICEPTR(float) *imageData_out, const CONSTANT(float) *imageData_in, int x, int y, Vector2i imgDims)
{
	float z, tmpz, dz, final_depth = 0.0f, w, w_sum = 0.0f;

	z = imageData_in[x + y * imgDims.x];
	if (z < 0.0f) { imageData_out[x + y * imgDims.x] = -1.0f; return; }
	float var_d = 200.f*200.f / (1.2f + 1.9f*(z - 0.2f)*(z - 0.2f) + 0.1f / sqrt(z)) * 0.5f;

	for (int i = -2, count = 0; i <= 2; i++) for (int j = -2; j <= 2; j++, count++)
	{
		tmpz = imageData_in[(x + j) + (y + i) * imgDims.x];
		if (tmpz < 0.0f) continue;
		dz = (tmpz - z); dz *= dz;
		if (dz > 100.0f) continue;
		w = conv[count] * expf(-dz*var_d);
		w_sum += w;
		final_depth += w*tmpz;
	}

	final_depth /= w_sum;
	imageData_out[x + y*imgDims.x] = final_depth;
}