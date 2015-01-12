// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#ifndef __METALC__

#include <ostream>

typedef unsigned char uchar;
typedef unsigned short ushort;
typedef unsigned int uint;
typedef unsigned long ulong;

#ifndef MIN
#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a,b) (((a) < (b)) ? (b) : (a))
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

#ifndef TO_INT_ROUND3
#define TO_INT_ROUND3(x) (x).toIntRound()
#endif

#ifndef TO_INT_ROUND4
#define TO_INT_ROUND4(x) (x).toIntRound()
#endif

#ifndef TO_INT_FLOOR3
#define TO_INT_FLOOR3(inted, coeffs, in) inted = (in).toIntFloor(coeffs)
#endif

#ifndef TO_SHORT_FLOOR3
#define TO_SHORT_FLOOR3(x) (x).toShortFloor()
#endif

#ifndef TO_UCHAR3
#define TO_UCHAR3(x) (x).toUChar()
#endif

#ifndef TO_FLOAT3
#define TO_FLOAT3(x) (x).toFloat()
#endif

#ifndef TO_VECTOR3
#define TO_VECTOR3(a) (a).toVector3()
#endif

#ifndef IS_EQUAL3
#define IS_EQUAL3(a,b) ((a) == (b))
#endif

#else

using namespace metal;

typedef float3x3 Matrix3f;
typedef float4x4 Matrix4f;

typedef short2 Vector2s;
typedef int2 Vector2i;
typedef float2 Vector2f;

typedef short3 Vector3s;
typedef int3 Vector3i;
typedef uint3 Vector3ui;
typedef uchar3 Vector3u;
typedef float3 Vector3f;

typedef float4 Vector4f;
typedef int4 Vector4i;
typedef short4 Vector4s;
typedef uchar4 Vector4u;

#ifndef MIN
#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a,b) (((a) < (b)) ? (b) : (a))
#endif

#ifndef TO_INT_ROUND3
#define TO_INT_ROUND3(x) (static_cast<int3>(round(x)))
#endif

#ifndef TO_INT_ROUND4
#define TO_INT_ROUND4(x) (static_cast<int4>(round(x)))
#endif

#ifndef TO_INT_FLOOR3
#define TO_INT_FLOOR3(inted, coeffs, in) { Vector3f flored(floor(in.x), floor(in.y), floor(in.z)); coeffs = in - flored; inted = Vector3i((int)flored.x, (int)flored.y, (int)flored.z); }
#endif

#ifndef CLAMP
#define CLAMP(x,a,b) MAX((a), MIN((b), (x)))
#endif

#ifndef TO_SHORT_FLOOR3
#define TO_SHORT_FLOOR3(x) (static_cast<short3>(floor(x)))
#endif

#ifndef TO_UCHAR3
#define TO_UCHAR3(x) (static_cast<uchar3>(x))
#endif

#ifndef TO_FLOAT3
#define TO_FLOAT3(x) (static_cast<float3>(x))
#endif

#ifndef TO_VECTOR3
#define TO_VECTOR3(a) ((a).xyz)
#endif

#ifndef IS_EQUAL3
#define IS_EQUAL3(a,b) (((a).x == (b).x) && ((a).y == (b).y) && ((a).z == (b).z))
#endif

#endif