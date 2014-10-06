// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once
#include <ostream>

typedef unsigned char uchar;
typedef unsigned short ushort;
typedef unsigned int uint;
typedef unsigned long ulong;

#ifndef MIN
#define MIN(a,b) ((a < b) ? a : b)
#endif

#ifndef MAX
#define MAX(a,b) ((a < b) ? b : a)
#endif

#ifndef ABS
#define ABS(a) ((a < 0) ? -a : a)
#endif

#ifndef CLAMP
#define CLAMP(x,a,b) MAX((a), MIN((b), (x)))
#endif

#ifndef ROUND
#define ROUND(x) ((x < 0) ? (x - 0.5f) : (x + 0.5f))
#endif

#ifndef PI
#define PI float(3.1415926535897932384626433832795)
#endif

#ifndef DEGTORAD
#define DEGTORAD float(0.017453292519943295769236907684886)
#endif

#ifndef MY_INF
#define MY_INF 0x7f800000
#endif

#ifndef NULL
#define NULL 0
#endif

#include "ITMVector.h"
#include "ITMMatrix.h"

typedef class ITMLib::Matrix3<float> Matrix3f;
typedef class ITMLib::Matrix4<float> Matrix4f;

typedef class ITMLib::Vector2<short> Vector2s;
typedef class ITMLib::Vector2<int> Vector2i;
typedef class ITMLib::Vector2<float> Vector2f;
typedef class ITMLib::Vector2<double> Vector2d;

typedef class ITMLib::Vector3<short> Vector3s;
typedef class ITMLib::Vector3<double> Vector3d;
typedef class ITMLib::Vector3<int> Vector3i;
typedef class ITMLib::Vector3<uint> Vector3ui;
typedef class ITMLib::Vector3<uchar> Vector3u;
typedef class ITMLib::Vector3<float> Vector3f;

typedef class ITMLib::Vector4<float> Vector4f;
typedef class ITMLib::Vector4<int> Vector4i;
typedef class ITMLib::Vector4<short> Vector4s;
typedef class ITMLib::Vector4<uchar> Vector4u;

inline bool portable_finite(float a)
{
	volatile float temp = a;
	if (temp != a) return false;
	if ((temp - a) != 0.0) return false;
	return true;
}

inline void matmul(const float *A, const float *b, float *x, int numRows, int numCols)
{
	for (int r = 0; r < numRows; ++r)
	{
		float res = 0.0f;
		for (int c = 0; c < numCols; ++c) res += A[r*numCols + c] * b[c];
		x[r] = res;
	}
}
