// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once
#ifndef __METALC__
#include <math.h>
#include <ostream>
#endif

#include "../../Utils/ITMLibDefines.h"

template<typename T> _CPU_AND_GPU_CODE_ inline Vector4f interpolateBilinear(const CONSTPTR(T) *source,
	const THREADPTR(Vector2f) & position, const CONSTPTR(Vector2i) & imgSize)
{
	T a, b, c, d; Vector4f result;
	Vector2i p; Vector2f delta;

	p.x = (int)floor(position.x); p.y = (int)floor(position.y);
	delta.x = position.x - (float)p.x; delta.y = position.y - (float)p.y;

	b.x = 0; b.y = 0; b.z = 0; b.w = 0;
	c.x = 0; c.y = 0; c.z = 0; c.w = 0;
	d.x = 0; d.y = 0; d.z = 0; d.w = 0;

	a = source[p.x + p.y * imgSize.x];
	if (delta.x != 0) b = source[(p.x + 1) + p.y * imgSize.x];
	if (delta.y != 0) c = source[p.x + (p.y + 1) * imgSize.x];
	if (delta.x != 0 && delta.y != 0) d = source[(p.x + 1) + (p.y + 1) * imgSize.x];

	result.x = ((float)a.x * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.x * delta.x * (1.0f - delta.y) +
		(float)c.x * (1.0f - delta.x) * delta.y + (float)d.x * delta.x * delta.y);
	result.y = ((float)a.y * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.y * delta.x * (1.0f - delta.y) +
		(float)c.y * (1.0f - delta.x) * delta.y + (float)d.y * delta.x * delta.y);
	result.z = ((float)a.z * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.z * delta.x * (1.0f - delta.y) +
		(float)c.z * (1.0f - delta.x) * delta.y + (float)d.z * delta.x * delta.y);
	result.w = ((float)a.w * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.w * delta.x * (1.0f - delta.y) +
		(float)c.w * (1.0f - delta.x) * delta.y + (float)d.w * delta.x * delta.y);

	return result;
}

template<typename T> _CPU_AND_GPU_CODE_ inline Vector4f interpolateBilinear_withHoles(const CONSTPTR(T) *source,
	const THREADPTR(Vector2f) & position, const CONSTPTR(Vector2i) & imgSize)
{
	T a, b, c, d; Vector4f result;
	Vector2s p; Vector2f delta;

	p.x = (short)floor(position.x); p.y = (short)floor(position.y);
	delta.x = position.x - (float)p.x; delta.y = position.y - (float)p.y;

	a = source[p.x + p.y * imgSize.x];
	b = source[(p.x + 1) + p.y * imgSize.x];
	c = source[p.x + (p.y + 1) * imgSize.x];
	d = source[(p.x + 1) + (p.y + 1) * imgSize.x];

	if (a.w < 0 || b.w < 0 || c.w < 0 || d.w < 0)
	{
		result.x = 0; result.y = 0; result.z = 0; result.w = -1.0f;
		return result;
	}

	result.x = ((float)a.x * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.x * delta.x * (1.0f - delta.y) +
		(float)c.x * (1.0f - delta.x) * delta.y + (float)d.x * delta.x * delta.y);
	result.y = ((float)a.y * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.y * delta.x * (1.0f - delta.y) +
		(float)c.y * (1.0f - delta.x) * delta.y + (float)d.y * delta.x * delta.y);
	result.z = ((float)a.z * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.z * delta.x * (1.0f - delta.y) +
		(float)c.z * (1.0f - delta.x) * delta.y + (float)d.z * delta.x * delta.y);
	result.w = ((float)a.w * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.w * delta.x * (1.0f - delta.y) +
		(float)c.w * (1.0f - delta.x) * delta.y + (float)d.w * delta.x * delta.y);

	return result;
}

template<typename T> _CPU_AND_GPU_CODE_ inline float interpolateBilinear_withHoles_single(const CONSTPTR(T) *source,
	const THREADPTR(Vector2f) & position, const CONSTPTR(Vector2i) & imgSize)
{
	T a = 0, b = 0, c = 0, d = 0; float result;
	Vector2i p; Vector2f delta;

	p.x = (int)floor(position.x); p.y = (int)floor(position.y);
	delta.x = position.x - (float)p.x; delta.y = position.y - (float)p.y;

	a = source[p.x + p.y * imgSize.x];
	if (delta.x != 0) b = source[(p.x + 1) + p.y * imgSize.x];
	if (delta.y != 0) c = source[p.x + (p.y + 1) * imgSize.x];
	if (delta.x != 0 && delta.y != 0) d = source[(p.x + 1) + (p.y + 1) * imgSize.x];

	if (a < 0 || b < 0 || c < 0 || d < 0) return -1;

	result = ((float)a * (1.0f - delta.x) * (1.0f - delta.y) + (float)b * delta.x * (1.0f - delta.y) +
		(float)c * (1.0f - delta.x) * delta.y + (float)d * delta.x * delta.y);

	return result;
}
