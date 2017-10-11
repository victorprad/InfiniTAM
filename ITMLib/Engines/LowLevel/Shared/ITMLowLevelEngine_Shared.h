// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Utils/ITMMath.h"

_CPU_AND_GPU_CODE_ inline void convertColourToIntensity(DEVICEPTR(float) *imageData_out, int x, int y, Vector2i dims,
		const CONSTPTR(Vector4u) *imageData_in)
{
	const int linear_pos = y * dims.x + x;
	const Vector4u colour = imageData_in[linear_pos];

	imageData_out[linear_pos] = (0.299f * colour.x + 0.587f * colour.y + 0.114f * colour.z) / 255.f;
}

_CPU_AND_GPU_CODE_ inline void filterSubsample(DEVICEPTR(Vector4u) *imageData_out, int x, int y, Vector2i newDims, 
	const CONSTPTR(Vector4u) *imageData_in, Vector2i oldDims)
{
	int src_pos_x = x * 2, src_pos_y = y * 2;
	Vector4u pixel_out, pixels_in[4];

	pixels_in[0] = imageData_in[(src_pos_x + 0) + (src_pos_y + 0) * oldDims.x];
	pixels_in[1] = imageData_in[(src_pos_x + 1) + (src_pos_y + 0) * oldDims.x];
	pixels_in[2] = imageData_in[(src_pos_x + 0) + (src_pos_y + 1) * oldDims.x];
	pixels_in[3] = imageData_in[(src_pos_x + 1) + (src_pos_y + 1) * oldDims.x];

	pixel_out.x = (pixels_in[0].x + pixels_in[1].x + pixels_in[2].x + pixels_in[3].x) / 4;
	pixel_out.y = (pixels_in[0].y + pixels_in[1].y + pixels_in[2].y + pixels_in[3].y) / 4;
	pixel_out.z = (pixels_in[0].z + pixels_in[1].z + pixels_in[2].z + pixels_in[3].z) / 4;
	pixel_out.w = (pixels_in[0].w + pixels_in[1].w + pixels_in[2].w + pixels_in[3].w) / 4;

	imageData_out[x + y * newDims.x] = pixel_out;
}

_CPU_AND_GPU_CODE_ inline void boxFilter2x2(DEVICEPTR(float) *imageData_out, int x_out, int y_out, Vector2i newDims,
	const CONSTPTR(float) *imageData_in, int x_in, int y_in, Vector2i oldDims)
{
	float pixel_out = 0.f;

	pixel_out += imageData_in[(x_in + 0) + (y_in + 0) * oldDims.x];
	pixel_out += imageData_in[(x_in + 1) + (y_in + 0) * oldDims.x];
	pixel_out += imageData_in[(x_in + 0) + (y_in + 1) * oldDims.x];
	pixel_out += imageData_in[(x_in + 1) + (y_in + 1) * oldDims.x];

	imageData_out[x_out + y_out * newDims.x] = pixel_out / 4.f;
}

_CPU_AND_GPU_CODE_ inline void filterSubsampleWithHoles(DEVICEPTR(float) *imageData_out, int x, int y, Vector2i newDims, 
	const CONSTPTR(float) *imageData_in, Vector2i oldDims)
{
	int src_pos_x = x * 2, src_pos_y = y * 2;
	float pixel_out = 0.0f, pixel_in, no_good_pixels = 0.0f;

	pixel_in = imageData_in[(src_pos_x + 0) + (src_pos_y + 0) * oldDims.x];
	if (pixel_in > 0.0f) { pixel_out += pixel_in; no_good_pixels++; }

	pixel_in = imageData_in[(src_pos_x + 1) + (src_pos_y + 0) * oldDims.x];
	if (pixel_in > 0.0f) { pixel_out += pixel_in; no_good_pixels++; }

	pixel_in = imageData_in[(src_pos_x + 0) + (src_pos_y + 1) * oldDims.x];
	if (pixel_in > 0.0f) { pixel_out += pixel_in; no_good_pixels++; }

	pixel_in = imageData_in[(src_pos_x + 1) + (src_pos_y + 1) * oldDims.x];
	if (pixel_in > 0.0f) { pixel_out += pixel_in; no_good_pixels++; }

	if (no_good_pixels > 0) pixel_out /= no_good_pixels;

	imageData_out[x + y * newDims.x] = pixel_out;
}

_CPU_AND_GPU_CODE_ inline void filterSubsampleWithHoles(DEVICEPTR(Vector4f) *imageData_out, int x, int y, Vector2i newDims, 
	const CONSTPTR(Vector4f) *imageData_in, Vector2i oldDims)
{
	int src_pos_x = x * 2, src_pos_y = y * 2;
	Vector4f pixel_out(0.0f), pixel_in; float no_good_pixels = 0.0f;

	pixel_in = imageData_in[(src_pos_x + 0) + (src_pos_y + 0) * oldDims.x];
	if (pixel_in.w >= 0) { pixel_out += pixel_in; no_good_pixels++; }

	pixel_in = imageData_in[(src_pos_x + 1) + (src_pos_y + 0) * oldDims.x];
	if (pixel_in.w >= 0) { pixel_out += pixel_in; no_good_pixels++; }

	pixel_in = imageData_in[(src_pos_x + 0) + (src_pos_y + 1) * oldDims.x];
	if (pixel_in.w >= 0) { pixel_out += pixel_in; no_good_pixels++; }

	pixel_in = imageData_in[(src_pos_x + 1) + (src_pos_y + 1) * oldDims.x];
	if (pixel_in.w >= 0) { pixel_out += pixel_in; no_good_pixels++; }

	if (no_good_pixels > 0) pixel_out /= no_good_pixels;
	else { pixel_out.w = -1.0f; }

	imageData_out[x + y * newDims.x] = pixel_out;
}

_CPU_AND_GPU_CODE_ inline void gradientX(DEVICEPTR(Vector4s) *grad, int x, int y, const CONSTPTR(Vector4u) *image, Vector2i imgSize)
{
	Vector4s d1, d2, d3, d_out;

	d1.x = image[(x + 1) + (y - 1) * imgSize.x].x - image[(x - 1) + (y - 1) * imgSize.x].x;
	d1.y = image[(x + 1) + (y - 1) * imgSize.x].y - image[(x - 1) + (y - 1) * imgSize.x].y;
	d1.z = image[(x + 1) + (y - 1) * imgSize.x].z - image[(x - 1) + (y - 1) * imgSize.x].z;

	d2.x = image[(x + 1) + (y)* imgSize.x].x - image[(x - 1) + (y)* imgSize.x].x;
	d2.y = image[(x + 1) + (y)* imgSize.x].y - image[(x - 1) + (y)* imgSize.x].y;
	d2.z = image[(x + 1) + (y)* imgSize.x].z - image[(x - 1) + (y)* imgSize.x].z;

	d3.x = image[(x + 1) + (y + 1) * imgSize.x].x - image[(x - 1) + (y + 1) * imgSize.x].x;
	d3.y = image[(x + 1) + (y + 1) * imgSize.x].y - image[(x - 1) + (y + 1) * imgSize.x].y;
	d3.z = image[(x + 1) + (y + 1) * imgSize.x].z - image[(x - 1) + (y + 1) * imgSize.x].z;

	d1.w = d2.w = d3.w = 2 * 255;

	d_out.x = (d1.x + 2 * d2.x + d3.x) / 8;
	d_out.y = (d1.y + 2 * d2.y + d3.y) / 8;
	d_out.z = (d1.z + 2 * d2.z + d3.z) / 8;
	d_out.w = (d1.w + 2 * d2.w + d3.w) / 8;

	grad[x + y * imgSize.x] = d_out;
}

_CPU_AND_GPU_CODE_ inline void gradientY(DEVICEPTR(Vector4s) *grad, int x, int y, const CONSTPTR(Vector4u) *image, Vector2i imgSize)
{
	Vector4s d1, d2, d3, d_out;

	d1.x = image[(x - 1) + (y + 1) * imgSize.x].x - image[(x - 1) + (y - 1) * imgSize.x].x;
	d1.y = image[(x - 1) + (y + 1) * imgSize.x].y - image[(x - 1) + (y - 1) * imgSize.x].y;
	d1.z = image[(x - 1) + (y + 1) * imgSize.x].z - image[(x - 1) + (y - 1) * imgSize.x].z;

	d2.x = image[(x)+(y + 1) * imgSize.x].x - image[(x)+(y - 1) * imgSize.x].x;
	d2.y = image[(x)+(y + 1) * imgSize.x].y - image[(x)+(y - 1) * imgSize.x].y;
	d2.z = image[(x)+(y + 1) * imgSize.x].z - image[(x)+(y - 1) * imgSize.x].z;

	d3.x = image[(x + 1) + (y + 1) * imgSize.x].x - image[(x + 1) + (y - 1) * imgSize.x].x;
	d3.y = image[(x + 1) + (y + 1) * imgSize.x].y - image[(x + 1) + (y - 1) * imgSize.x].y;
	d3.z = image[(x + 1) + (y + 1) * imgSize.x].z - image[(x + 1) + (y - 1) * imgSize.x].z;

	d1.w = d2.w = d3.w = 2 * 255;

	d_out.x = (d1.x + 2 * d2.x + d3.x) / 8;
	d_out.y = (d1.y + 2 * d2.y + d3.y) / 8;
	d_out.z = (d1.z + 2 * d2.z + d3.z) / 8;
	d_out.w = (d1.w + 2 * d2.w + d3.w) / 8;

	grad[x + y * imgSize.x] = d_out;
}

_CPU_AND_GPU_CODE_ inline void gradientXY(DEVICEPTR(Vector2f) *grad, int x, int y, const CONSTPTR(float) *image, Vector2i imgSize)
{
	Vector2f d1, d2, d3, d_out;

	// Compute gradient in the X direction
	d1.x = image[(y - 1) * imgSize.x + (x + 1)] - image[(y - 1) * imgSize.x + (x - 1)];
	d2.x = image[(y    ) * imgSize.x + (x + 1)] - image[(y    ) * imgSize.x + (x - 1)];
	d3.x = image[(y + 1) * imgSize.x + (x + 1)] - image[(y + 1) * imgSize.x + (x - 1)];

	// Compute gradient in the Y direction
	d1.y = image[(y + 1) * imgSize.x + (x - 1)] - image[(y - 1) * imgSize.x + (x - 1)];
	d2.y = image[(y + 1) * imgSize.x + (x    )] - image[(y - 1) * imgSize.x + (x    )];
	d3.y = image[(y + 1) * imgSize.x + (x + 1)] - image[(y - 1) * imgSize.x + (x + 1)];

	d_out.x = (d1.x + 2.f * d2.x + d3.x) / 8.f;
	d_out.y = (d1.y + 2.f * d2.y + d3.y) / 8.f;

	grad[y * imgSize.x + x] = d_out;
}
