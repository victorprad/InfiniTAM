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

_CPU_AND_GPU_CODE_ inline void smoothingRawDepth(DEVICEPTR(float) *imageData_out, const CONSTANT(float) *imageData_in, int x, int y, Vector2i imgDims, Vector3f zdirc)
{
	Vector2f patch[9];
	float no_good_pixels = 0.0f, var_d = 0.0f, var_u = 0.0f;
	Vector3f normal_x, normal_y, normal_z;

	float z, du, dz, final_depth, w, w_sum;

	if (imageData_in[x + y * imgDims.x] < 0.0f) { imageData_out[x + y * imgDims.x] = -1.0f; return; }

	for (int i = -1, count = 0; i <= 1; i++) for (int j = -1; j <= 1; j++, count++)
	{
		patch[count].x = imageData_in[(x + j) + (y + i) * imgDims.x];
		if (patch[count].x > 0.0f) patch[count].y = i*i + j*j;
	}

	z = patch[4].x;
	if (no_good_pixels == 1.0f){ imageData_out[x + y * imgDims.x] = z; return; }
	if (patch[1].x == -1 || patch[3].x == -1 || patch[5].x == -1 || patch[7].x == -1){ imageData_out[x + y * imgDims.x] = z; return; }

	normal_x.x = 1.0f; normal_x.y = 0.0f; normal_x.z = patch[5].x - patch[3].x;
	normal_y.x = 0.0f; normal_y.y = 1.0f; normal_y.z = patch[7].x - patch[1].x;
	
	normal_z.x = normal_x.y*normal_y.z - normal_y.y*normal_x.z;
	normal_z.y = normal_x.z*normal_y.x - normal_y.z*normal_x.x;
	normal_z.z = normal_x.x*normal_y.y - normal_y.x*normal_x.y;
	normal_z = normal_z.normalised();
	zdirc = zdirc.normalised();

	float theta = fabs(acosf(-(normal_z.x*zdirc.x + normal_z.y*zdirc.y + normal_z.z*zdirc.z)));
	if (theta > 1.2217f) { imageData_out[x + y * imgDims.x] = -1.0f; return; }

	float diffPI = (PI / 2 - theta);
	var_u = 0.8f + 0.035f*(theta / diffPI);
	var_d = 0.0012f + 0.0019f*(z - 0.4f)*(z - 0.4f) + 0.0001f / sqrt(z) * theta * theta / diffPI / diffPI;


	w_sum = 0, final_depth = 0; // recycle this var for weight

	for (int i = 0; i < 9; i++)
	{
		if (patch[i].x>0)
		{
			dz = patch[i].x - z;
			//if (dz>0.0005f) continue;
			dz /= var_d;
			du = patch[i].y/var_u/var_u;
			w = expf(-0.5f * dz * dz - 0.5f * du);
			w_sum += w;
			final_depth += w*patch[i].x;
		}
	}

	final_depth /= w_sum;

	if (w_sum>0.001f)imageData_out[x + y*imgDims.x] = final_depth;
	else imageData_out[x + y*imgDims.x] = z;
}