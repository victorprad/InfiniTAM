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

_CPU_AND_GPU_CODE_ inline void smoothingRawDepth(DEVICEPTR(float) *imageData_out, const CONSTANT(float) *imageData_in, int x, int y, Vector2i imgDims)
{
	Vector2f patch[9];
	float no_good_pixels = 0.0f, mean_d = 0.0f, mean_u = 0.0f, var_d = 0.0f, var_u = 0.0f;
	float du, dz, tmpu, final_depth, w;

	if (imageData_in[x + y * imgDims.x] < 0.0f) { imageData_out[x + y * imgDims.x] = -1.0f; return; }

	for (int i = -1, count = 0; i <= 1; i++) for (int j = -1; j <= 1; j++, count++)
	{
		patch[count].x = imageData_in[(x + j) + (y + i) * imgDims.x];
		if (patch[count].x > 0.0f)
		{
			mean_d += patch[count].x;
			var_d += patch[count].x * patch[count].x;

			tmpu = i*i + j*j;
			mean_u += sqrt(tmpu);
			var_u += tmpu;
			patch[count].y = tmpu;

			no_good_pixels++;
		}
	}

	if (no_good_pixels == 0.0f){ imageData_out[x + y * imgDims.x] = -1; return; }

	mean_d /= no_good_pixels;
	mean_u /= no_good_pixels;
	var_d = var_d / no_good_pixels - mean_d*mean_d;
	var_u = var_u / no_good_pixels - mean_u*mean_u;

	tmpu = 0, final_depth = 0; // recycle this var for weight

	for (int i = 0; i < 9; i++)
	{
		if (patch[i].x>0)
		{
			//dz = patch[i].x - patch[4].x; dz *= dz;
			dz = patch[i].x - mean_d; dz *= dz;
			if (dz>0.0005f) continue;
			du = patch[i].y;
			w = expf(-0.5f * dz / var_d - 0.5f * du / var_u);
			tmpu += w;
			final_depth += w*patch[i].x;
		}
	}

	if (tmpu>0.001f)imageData_out[x + y*imgDims.x] = final_depth / tmpu;
	else imageData_out[x + y*imgDims.x] = mean_d;
}