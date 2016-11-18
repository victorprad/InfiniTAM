// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Utils/ITMMath.h"
#include "../../../../ORUtils/PlatformIndependence.h"

_CPU_AND_GPU_CODE_ inline void convertDisparityToDepth(DEVICEPTR(float) *d_out, int x, int y, const CONSTPTR(short) *d_in,
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

#ifndef __METALC__

_CPU_AND_GPU_CODE_ inline void convertDepthAffineToFloat(DEVICEPTR(float) *d_out, int x, int y, const CONSTPTR(short) *d_in, Vector2i imgSize, Vector2f depthCalibParams)
{
	int locId = x + y * imgSize.x;

	short depth_in = d_in[locId];
	d_out[locId] = ((depth_in <= 0)||(depth_in > 32000)) ? -1.0f : (float)depth_in * depthCalibParams.x + depthCalibParams.y;
}

#define MEAN_SIGMA_L 1.2232f
_CPU_AND_GPU_CODE_ inline void filterDepth(DEVICEPTR(float) *imageData_out, const CONSTPTR(float) *imageData_in, int x, int y, Vector2i imgDims)
{
	float z, tmpz, dz, final_depth = 0.0f, w, w_sum = 0.0f;

	z = imageData_in[x + y * imgDims.x];
	if (z < 0.0f) { imageData_out[x + y * imgDims.x] = -1.0f; return; }

	float sigma_z = 1.0f / (0.0012f + 0.0019f*(z - 0.4f)*(z - 0.4f) + 0.0001f / sqrt(z) * 0.25f);

	for (int i = -2, count = 0; i <= 2; i++) for (int j = -2; j <= 2; j++, count++)
	{
		tmpz = imageData_in[(x + j) + (y + i) * imgDims.x];
		if (tmpz < 0.0f) continue;
		dz = (tmpz - z); dz *= dz;
		w = exp(-0.5f * ((abs(i) + abs(j))*MEAN_SIGMA_L*MEAN_SIGMA_L + dz * sigma_z * sigma_z));
		w_sum += w;
		final_depth += w*tmpz;
	}

	final_depth /= w_sum;
	imageData_out[x + y*imgDims.x] = final_depth;
}


_CPU_AND_GPU_CODE_ inline void computeNormalAndWeight(const CONSTPTR(float) *depth_in, DEVICEPTR(Vector4f) *normal_out, DEVICEPTR(float) *sigmaZ_out, int x, int y, Vector2i imgDims, Vector4f intrinparam)
{
	Vector3f outNormal;

	int idx = x + y * imgDims.x;

	float z = depth_in[x + y * imgDims.x];
	if (z < 0.0f)
	{
		normal_out[idx].w = -1.0f;
		sigmaZ_out[idx] = -1;
		return;
	}

	// first compute the normal
	Vector3f xp1_y, xm1_y, x_yp1, x_ym1;
	Vector3f diff_x(0.0f, 0.0f, 0.0f), diff_y(0.0f, 0.0f, 0.0f);

	xp1_y.z = depth_in[(x + 1) + y * imgDims.x], x_yp1.z = depth_in[x + (y + 1) * imgDims.x];
	xm1_y.z = depth_in[(x - 1) + y * imgDims.x], x_ym1.z = depth_in[x + (y - 1) * imgDims.x];

	if (xp1_y.z <= 0 || x_yp1.z <= 0 || xm1_y.z <= 0 || x_ym1.z <= 0)
	{
		normal_out[idx].w = -1.0f;
		sigmaZ_out[idx] = -1;
		return;
	}

	// unprojected
	xp1_y.x = xp1_y.z * ((x + 1.0f) - intrinparam.z) * intrinparam.x; xp1_y.y = xp1_y.z * (y - intrinparam.w) * intrinparam.y;
	xm1_y.x = xm1_y.z * ((x - 1.0f) - intrinparam.z) * intrinparam.x; xm1_y.y = xm1_y.z * (y - intrinparam.w) * intrinparam.y;
	x_yp1.x = x_yp1.z * (x - intrinparam.z) * intrinparam.x; x_yp1.y = x_yp1.z * ((y + 1.0f) - intrinparam.w) * intrinparam.y;
	x_ym1.x = x_ym1.z * (x - intrinparam.z) * intrinparam.x; x_ym1.y = x_ym1.z * ((y - 1.0f) - intrinparam.w) * intrinparam.y;

	// gradients x and y
	diff_x = xp1_y - xm1_y, diff_y = x_yp1 - x_ym1;

	// cross product
	outNormal.x = (diff_x.y * diff_y.z - diff_x.z*diff_y.y);
	outNormal.y = (diff_x.z * diff_y.x - diff_x.x*diff_y.z);
	outNormal.z = (diff_x.x * diff_y.y - diff_x.y*diff_y.x);

	if (outNormal.x == 0.0f && outNormal.y == 0 && outNormal.z == 0)
	{
		normal_out[idx].w = -1.0f;
		sigmaZ_out[idx] = -1;
		return;
	}

    float norm = 1.0f / sqrt(outNormal.x * outNormal.x + outNormal.y * outNormal.y + outNormal.z * outNormal.z);
    outNormal *= norm;
    
	normal_out[idx].x = outNormal.x; normal_out[idx].y = outNormal.y; normal_out[idx].z = outNormal.z; normal_out[idx].w = 1.0f;

	// now compute weight
	float theta = acos(outNormal.z);
	float theta_diff = theta / (PI*0.5f - theta);

	sigmaZ_out[idx] = (0.0012f + 0.0019f * (z - 0.4f) * (z - 0.4f) + 0.0001f / sqrt(z) * theta_diff * theta_diff);
}

#endif
