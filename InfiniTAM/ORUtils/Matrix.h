// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM
#pragma once

#include <string.h>
#include <ostream>

#include "PlatformIndependence.h"
#include "Vector.h"

/************************************************************************/
/* WARNING: the following 3x3 and 4x4 matrix are using column major, to	*/
/* be consistent with OpenGL default rather than most C/C++ default.	*/
/* In all other parts of the code, we still use row major order.		*/
/************************************************************************/

namespace ORUtils {
	template <class T> class Vector2;
	template <class T> class Vector3;
	template <class T> class Vector4;
	template <class T, int s> class VectorX;

	//////////////////////////////////////////////////////////////////////////
	//						Basic Matrix Structure
	//////////////////////////////////////////////////////////////////////////

	template <class T> struct Matrix4_{
		union {
			struct { // Warning: see the header in this file for the special matrix order
				T m00, m01, m02, m03;	// |0, 4, 8,  12|    |m00, m10, m20, m30|
				T m10, m11, m12, m13;	// |1, 5, 9,  13|    |m01, m11, m21, m31|
				T m20, m21, m22, m23;	// |2, 6, 10, 14|    |m02, m12, m22, m32|
				T m30, m31, m32, m33;	// |3, 7, 11, 15|    |m03, m13, m23, m33|
			};
			T m[16];
		};
	};

	template <class T> struct Matrix3_{
		union { // Warning: see the header in this file for the special matrix order
			struct {
				T m00, m01, m02; // |0, 3, 6|     |m00, m10, m20|
				T m10, m11, m12; // |1, 4, 7|     |m01, m11, m21|
				T m20, m21, m22; // |2, 5, 8|     |m02, m12, m22|
			};
			T m[9];
		};
	};

	template<class T, int s> struct MatrixSQX_{
		int dim;
		int sq;
		T m[s*s];
	};

	//////////////////////////////////////////////////////////////////////////
	// Matrix class with math operators
	//////////////////////////////////////////////////////////////////////////
	template<class T>
	class Matrix4 : public Matrix4_ < T >
	{
	public:
		_CPU_AND_GPU_CODE_ Matrix4() {}
		_CPU_AND_GPU_CODE_ Matrix4(T t) { setValues(t); }
		_CPU_AND_GPU_CODE_ Matrix4(const T *m)	{ setValues(m); }
		_CPU_AND_GPU_CODE_ Matrix4(T a00, T a01, T a02, T a03, T a10, T a11, T a12, T a13, T a20, T a21, T a22, T a23, T a30, T a31, T a32, T a33)	{
			this->m00 = a00; this->m01 = a01; this->m02 = a02; this->m03 = a03;
			this->m10 = a10; this->m11 = a11; this->m12 = a12; this->m13 = a13;
			this->m20 = a20; this->m21 = a21; this->m22 = a22; this->m23 = a23;
			this->m30 = a30; this->m31 = a31; this->m32 = a32; this->m33 = a33;
		}

		_CPU_AND_GPU_CODE_ inline void getValues(T *mp) const	{ memcpy(mp, this->m, sizeof(T) * 16); }
		_CPU_AND_GPU_CODE_ inline const T *getValues() const { return this->m; }
		_CPU_AND_GPU_CODE_ inline Vector3<T> getScale() const { return Vector3<T>(this->m00, this->m11, this->m22); }

		// Element access
		_CPU_AND_GPU_CODE_ inline T &operator()(int x, int y)	{ return at(x, y); }
		_CPU_AND_GPU_CODE_ inline const T &operator()(int x, int y) const	{ return at(x, y); }
		_CPU_AND_GPU_CODE_ inline T &operator()(Vector2<int> pnt)	{ return at(pnt.x, pnt.y); }
		_CPU_AND_GPU_CODE_ inline const T &operator()(Vector2<int> pnt) const	{ return at(pnt.x, pnt.y); }
		_CPU_AND_GPU_CODE_ inline T &at(int x, int y) { return this->m[y | (x << 2)]; }
		_CPU_AND_GPU_CODE_ inline const T &at(int x, int y) const { return this->m[y | (x << 2)]; }

		// set values
		_CPU_AND_GPU_CODE_ inline void setValues(const T *mp) { memcpy(this->m, mp, sizeof(T) * 16); }
		_CPU_AND_GPU_CODE_ inline void setValues(T r)	{ for (int i = 0; i < 16; i++)	this->m[i] = r; }
		_CPU_AND_GPU_CODE_ inline void setZeros() { memset(this->m, 0, sizeof(T) * 16); }
		_CPU_AND_GPU_CODE_ inline void setIdentity() { setZeros(); this->m00 = this->m11 = this->m22 = this->m33 = 1; }
		_CPU_AND_GPU_CODE_ inline void setScale(T s) { this->m00 = this->m11 = this->m22 = s; }
		_CPU_AND_GPU_CODE_ inline void setScale(const Vector3_<T> &s) { this->m00 = s[0]; this->m11 = s[1]; this->m22 = s[2]; }
		_CPU_AND_GPU_CODE_ inline void setTranslate(const Vector3_<T> &t) { for (int y = 0; y < 3; y++) at(3, y) = t.v[y]; }
		_CPU_AND_GPU_CODE_ inline void setRow(int r, const Vector4_<T> &t){ for (int x = 0; x < 4; x++) at(x, r) = t.v[x]; }
		_CPU_AND_GPU_CODE_ inline void setColumn(int c, const Vector4_<T> &t) { memcpy(this->m + 4 * c, t.v, sizeof(T) * 4); }

		// get values
		_CPU_AND_GPU_CODE_ inline Vector4<T> getRow(int r) const { Vector4<T> v; for (int x = 0; x < 4; x++) v[x] = at(x, r); return v; }
		_CPU_AND_GPU_CODE_ inline Vector4<T> getColumn(int c) const { Vector4<T> v; memcpy(v.v, this->m + 4 * c, sizeof(T) * 4); return v; }
		_CPU_AND_GPU_CODE_ inline Matrix4 t() const { // transpose
			Matrix4 mtrans;
			for (int x = 0; x < 4; x++)	for (int y = 0; y < 4; y++)
				mtrans(x, y) = at(y, x);
			return mtrans;
		}

		_CPU_AND_GPU_CODE_ inline friend Matrix4 operator * (const Matrix4 &lhs, const T &rhs)	{ 
			Matrix4 r;
			for (int i = 0; i < 16; i++) r.m[i] = lhs.m[i] * rhs;
			return r;
		}

		_CPU_AND_GPU_CODE_ inline friend Matrix4 operator / (const Matrix4 &lhs, const T &rhs)	{ 
			Matrix4 r;
			for (int i = 0; i < 16; i++) r.m[i] = lhs.m[i] / rhs;
			return r;
		}

		_CPU_AND_GPU_CODE_ inline friend Matrix4 operator * (const Matrix4 &lhs, const Matrix4 &rhs)	{ 
			Matrix4 r;
			r.setZeros();
			for (int x = 0; x < 4; x++) for (int y = 0; y < 4; y++) for (int k = 0; k < 4; k++)
				r(x, y) += lhs(k, y) * rhs(x, k);
			return r;
		}

		_CPU_AND_GPU_CODE_ inline friend Matrix4 operator + (const Matrix4 &lhs, const Matrix4 &rhs) {
			Matrix4 res(lhs.m);
			return res += rhs;
		}

		_CPU_AND_GPU_CODE_ inline Vector4<T> operator *(const Vector4<T> &rhs) const {
			Vector4<T> r;
			r[0] = this->m[0] * rhs[0] + this->m[4] * rhs[1] + this->m[8] * rhs[2] + this->m[12] * rhs[3];
			r[1] = this->m[1] * rhs[0] + this->m[5] * rhs[1] + this->m[9] * rhs[2] + this->m[13] * rhs[3];
			r[2] = this->m[2] * rhs[0] + this->m[6] * rhs[1] + this->m[10] * rhs[2] + this->m[14] * rhs[3];
			r[3] = this->m[3] * rhs[0] + this->m[7] * rhs[1] + this->m[11] * rhs[2] + this->m[15] * rhs[3];
			return r;
		}

		// Used as a projection matrix to multiply with the Vector3
		_CPU_AND_GPU_CODE_ inline Vector3<T> operator *(const Vector3<T> &rhs) const {
			Vector3<T> r;
			r[0] = this->m[0] * rhs[0] + this->m[4] * rhs[1] + this->m[8] * rhs[2] + this->m[12];
			r[1] = this->m[1] * rhs[0] + this->m[5] * rhs[1] + this->m[9] * rhs[2] + this->m[13];
			r[2] = this->m[2] * rhs[0] + this->m[6] * rhs[1] + this->m[10] * rhs[2] + this->m[14];
			return r;
		}

		_CPU_AND_GPU_CODE_ inline friend Vector4<T> operator *(const Vector4<T> &lhs, const Matrix4 &rhs){
			Vector4<T> r;
			for (int x = 0; x < 4; x++)
				r[x] = lhs[0] * rhs(x, 0) + lhs[1] * rhs(x, 1) + lhs[2] * rhs(x, 2) + lhs[3] * rhs(x, 3);
			return r;
		}

		_CPU_AND_GPU_CODE_ inline Matrix4& operator += (const T &r) { for (int i = 0; i < 16; ++i) this->m[i] += r; return *this; }
		_CPU_AND_GPU_CODE_ inline Matrix4& operator -= (const T &r) { for (int i = 0; i < 16; ++i) this->m[i] -= r; return *this; }
		_CPU_AND_GPU_CODE_ inline Matrix4& operator *= (const T &r) { for (int i = 0; i < 16; ++i) this->m[i] *= r; return *this; }
		_CPU_AND_GPU_CODE_ inline Matrix4& operator /= (const T &r) { for (int i = 0; i < 16; ++i) this->m[i] /= r; return *this; }
		_CPU_AND_GPU_CODE_ inline Matrix4 &operator += (const Matrix4 &mat) { for (int i = 0; i < 16; ++i) this->m[i] += mat.m[i]; return *this; }
		_CPU_AND_GPU_CODE_ inline Matrix4 &operator -= (const Matrix4 &mat) { for (int i = 0; i < 16; ++i) this->m[i] -= mat.m[i]; return *this; }

		_CPU_AND_GPU_CODE_ inline friend bool operator == (const Matrix4 &lhs, const Matrix4 &rhs) {
			bool r = lhs.m[0] == rhs.m[0];
			for (int i = 1; i < 16; i++)
				r &= lhs.m[i] == rhs.m[i];
			return r;
		}

		_CPU_AND_GPU_CODE_ inline friend bool operator != (const Matrix4 &lhs, const Matrix4 &rhs) {
			bool r = lhs.m[0] != rhs.m[0];
			for (int i = 1; i < 16; i++)
				r |= lhs.m[i] != rhs.m[i];
			return r;
		}

		// The inverse matrix for float/double type
		_CPU_AND_GPU_CODE_ inline bool inv(Matrix4 &out) const {
			T tmp[12], src[16], det;
			T *dst = out.m;
			for (int i = 0; i < 4; i++) {
				src[i] = this->m[i * 4];
				src[i + 4] = this->m[i * 4 + 1];
				src[i + 8] = this->m[i * 4 + 2];
				src[i + 12] = this->m[i * 4 + 3];
			}

			tmp[0] = src[10] * src[15];
			tmp[1] = src[11] * src[14];
			tmp[2] = src[9] * src[15];
			tmp[3] = src[11] * src[13];
			tmp[4] = src[9] * src[14];
			tmp[5] = src[10] * src[13];
			tmp[6] = src[8] * src[15];
			tmp[7] = src[11] * src[12];
			tmp[8] = src[8] * src[14];
			tmp[9] = src[10] * src[12];
			tmp[10] = src[8] * src[13];
			tmp[11] = src[9] * src[12];

			dst[0] = (tmp[0] * src[5] + tmp[3] * src[6] + tmp[4] * src[7]) - (tmp[1] * src[5] + tmp[2] * src[6] + tmp[5] * src[7]);
			dst[1] = (tmp[1] * src[4] + tmp[6] * src[6] + tmp[9] * src[7]) - (tmp[0] * src[4] + tmp[7] * src[6] + tmp[8] * src[7]);
			dst[2] = (tmp[2] * src[4] + tmp[7] * src[5] + tmp[10] * src[7]) - (tmp[3] * src[4] + tmp[6] * src[5] + tmp[11] * src[7]);
			dst[3] = (tmp[5] * src[4] + tmp[8] * src[5] + tmp[11] * src[6]) - (tmp[4] * src[4] + tmp[9] * src[5] + tmp[10] * src[6]);

			det = src[0] * dst[0] + src[1] * dst[1] + src[2] * dst[2] + src[3] * dst[3];
			if (det == 0.0f)
				return false;

			dst[4] = (tmp[1] * src[1] + tmp[2] * src[2] + tmp[5] * src[3]) - (tmp[0] * src[1] + tmp[3] * src[2] + tmp[4] * src[3]);
			dst[5] = (tmp[0] * src[0] + tmp[7] * src[2] + tmp[8] * src[3]) - (tmp[1] * src[0] + tmp[6] * src[2] + tmp[9] * src[3]);
			dst[6] = (tmp[3] * src[0] + tmp[6] * src[1] + tmp[11] * src[3]) - (tmp[2] * src[0] + tmp[7] * src[1] + tmp[10] * src[3]);
			dst[7] = (tmp[4] * src[0] + tmp[9] * src[1] + tmp[10] * src[2]) - (tmp[5] * src[0] + tmp[8] * src[1] + tmp[11] * src[2]);

			tmp[0] = src[2] * src[7];
			tmp[1] = src[3] * src[6];
			tmp[2] = src[1] * src[7];
			tmp[3] = src[3] * src[5];
			tmp[4] = src[1] * src[6];
			tmp[5] = src[2] * src[5];
			tmp[6] = src[0] * src[7];
			tmp[7] = src[3] * src[4];
			tmp[8] = src[0] * src[6];
			tmp[9] = src[2] * src[4];
			tmp[10] = src[0] * src[5];
			tmp[11] = src[1] * src[4];

			dst[8] = (tmp[0] * src[13] + tmp[3] * src[14] + tmp[4] * src[15]) - (tmp[1] * src[13] + tmp[2] * src[14] + tmp[5] * src[15]);
			dst[9] = (tmp[1] * src[12] + tmp[6] * src[14] + tmp[9] * src[15]) - (tmp[0] * src[12] + tmp[7] * src[14] + tmp[8] * src[15]);
			dst[10] = (tmp[2] * src[12] + tmp[7] * src[13] + tmp[10] * src[15]) - (tmp[3] * src[12] + tmp[6] * src[13] + tmp[11] * src[15]);
			dst[11] = (tmp[5] * src[12] + tmp[8] * src[13] + tmp[11] * src[14]) - (tmp[4] * src[12] + tmp[9] * src[13] + tmp[10] * src[14]);
			dst[12] = (tmp[2] * src[10] + tmp[5] * src[11] + tmp[1] * src[9]) - (tmp[4] * src[11] + tmp[0] * src[9] + tmp[3] * src[10]);
			dst[13] = (tmp[8] * src[11] + tmp[0] * src[8] + tmp[7] * src[10]) - (tmp[6] * src[10] + tmp[9] * src[11] + tmp[1] * src[8]);
			dst[14] = (tmp[6] * src[9] + tmp[11] * src[11] + tmp[3] * src[8]) - (tmp[10] * src[11] + tmp[2] * src[8] + tmp[7] * src[9]);
			dst[15] = (tmp[10] * src[10] + tmp[4] * src[8] + tmp[9] * src[9]) - (tmp[8] * src[9] + tmp[11] * src[10] + tmp[5] * src[8]);

			out *= 1 / det;
			return true;
		}

		friend std::ostream& operator<<(std::ostream& os, const Matrix4<T>& dt) {
			for (int y = 0; y < 4; y++)
				os << dt(0, y) << ", " << dt(1, y) << ", " << dt(2, y) << ", " << dt(3, y) << "\n";
			return os;
		}
	};

	template<class T>
	class Matrix3 : public Matrix3_ < T >
	{
	public:
		_CPU_AND_GPU_CODE_ Matrix3() {}
		_CPU_AND_GPU_CODE_ Matrix3(T t) { setValues(t); }
		_CPU_AND_GPU_CODE_ Matrix3(const T *m)	{ setValues(m); }
		_CPU_AND_GPU_CODE_ Matrix3(T a00, T a01, T a02, T a10, T a11, T a12, T a20, T a21, T a22)	{
			this->m00 = a00; this->m01 = a01; this->m02 = a02;
			this->m10 = a10; this->m11 = a11; this->m12 = a12;
			this->m20 = a20; this->m21 = a21; this->m22 = a22;
		}

		_CPU_AND_GPU_CODE_ inline void getValues(T *mp) const	{ memcpy(mp, this->m, sizeof(T) * 9); }
		_CPU_AND_GPU_CODE_ inline const T *getValues() const { return this->m; }
		_CPU_AND_GPU_CODE_ inline Vector3<T> getScale() const { return Vector3<T>(this->m00, this->m11, this->m22); }

		// Element access
		_CPU_AND_GPU_CODE_ inline T &operator()(int x, int y)	{ return at(x, y); }
		_CPU_AND_GPU_CODE_ inline const T &operator()(int x, int y) const	{ return at(x, y); }
		_CPU_AND_GPU_CODE_ inline T &operator()(Vector2<int> pnt)	{ return at(pnt.x, pnt.y); }
		_CPU_AND_GPU_CODE_ inline const T &operator()(Vector2<int> pnt) const	{ return at(pnt.x, pnt.y); }
		_CPU_AND_GPU_CODE_ inline T &at(int x, int y) { return this->m[x * 3 + y]; }
		_CPU_AND_GPU_CODE_ inline const T &at(int x, int y) const { return this->m[x * 3 + y]; }

		// set values
		_CPU_AND_GPU_CODE_ inline void setValues(const T *mp) { memcpy(this->m, mp, sizeof(T) * 9); }
		_CPU_AND_GPU_CODE_ inline void setValues(const T r)	{ for (int i = 0; i < 9; i++)	this->m[i] = r; }
		_CPU_AND_GPU_CODE_ inline void setZeros() { memset(this->m, 0, sizeof(T) * 9); }
		_CPU_AND_GPU_CODE_ inline void setIdentity() { setZeros(); this->m00 = this->m11 = this->m22 = 1; }
		_CPU_AND_GPU_CODE_ inline void setScale(T s) { this->m00 = this->m11 = this->m22 = s; }
		_CPU_AND_GPU_CODE_ inline void setScale(const Vector3_<T> &s) { this->m00 = s[0]; this->m11 = s[1]; this->m22 = s[2]; }
		_CPU_AND_GPU_CODE_ inline void setRow(int r, const Vector3_<T> &t){ for (int x = 0; x < 3; x++) at(x, r) = t[x]; }
		_CPU_AND_GPU_CODE_ inline void setColumn(int c, const Vector3_<T> &t) { memcpy(this->m + 3 * c, t.v, sizeof(T) * 3); }

		// get values
		_CPU_AND_GPU_CODE_ inline Vector3<T> getRow(int r) const { Vector3<T> v; for (int x = 0; x < 3; x++) v[x] = at(x, r); return v; }
		_CPU_AND_GPU_CODE_ inline Vector3<T> getColumn(int c) const { Vector3<T> v; memcpy(v.v, this->m + 3 * c, sizeof(T) * 3); return v; }
		_CPU_AND_GPU_CODE_ inline Matrix3 t() { // transpose
			Matrix3 mtrans;
			for (int x = 0; x < 3; x++)	for (int y = 0; y < 3; y++)
				mtrans(x, y) = at(y, x);
			return mtrans;
		}

		_CPU_AND_GPU_CODE_ inline friend Matrix3 operator * (const Matrix3 &lhs, const Matrix3 &rhs)	{
			Matrix3 r;
			r.setZeros();
			for (int x = 0; x < 3; x++) for (int y = 0; y < 3; y++) for (int k = 0; k < 3; k++)
				r(x, y) += lhs(k, y) * rhs(x, k);
			return r;
		}

		_CPU_AND_GPU_CODE_ inline friend Matrix3 operator + (const Matrix3 &lhs, const Matrix3 &rhs) {
			Matrix3 res(lhs.m);
			return res += rhs;
		}

		_CPU_AND_GPU_CODE_ inline Vector3<T> operator *(const Vector3<T> &rhs) const {
			Vector3<T> r;
			r[0] = this->m[0] * rhs[0] + this->m[3] * rhs[1] + this->m[6] * rhs[2];
			r[1] = this->m[1] * rhs[0] + this->m[4] * rhs[1] + this->m[7] * rhs[2];
			r[2] = this->m[2] * rhs[0] + this->m[5] * rhs[1] + this->m[8] * rhs[2];
			return r;
		}

		_CPU_AND_GPU_CODE_ inline Matrix3 operator *(const T &r) const {
			Matrix3 res(this->m);
			return res *= r;
		}

		_CPU_AND_GPU_CODE_ inline friend Vector3<T> operator *(const Vector3<T> &lhs, const Matrix3 &rhs){
			Vector3<T> r;
			for (int x = 0; x < 3; x++)
				r[x] = lhs[0] * rhs(x, 0) + lhs[1] * rhs(x, 1) + lhs[2] * rhs(x, 2);
			return r;
		}

		_CPU_AND_GPU_CODE_ inline Matrix3& operator += (const T &r) { for (int i = 0; i < 9; ++i) this->m[i] += r; return *this; }
		_CPU_AND_GPU_CODE_ inline Matrix3& operator -= (const T &r) { for (int i = 0; i < 9; ++i) this->m[i] -= r; return *this; }
		_CPU_AND_GPU_CODE_ inline Matrix3& operator *= (const T &r) { for (int i = 0; i < 9; ++i) this->m[i] *= r; return *this; }
		_CPU_AND_GPU_CODE_ inline Matrix3& operator /= (const T &r) { for (int i = 0; i < 9; ++i) this->m[i] /= r; return *this; }
		_CPU_AND_GPU_CODE_ inline Matrix3& operator += (const Matrix3 &mat) { for (int i = 0; i < 9; ++i) this->m[i] += mat.m[i]; return *this; }
		_CPU_AND_GPU_CODE_ inline Matrix3& operator -= (const Matrix3 &mat) { for (int i = 0; i < 9; ++i) this->m[i] -= mat.m[i]; return *this; }

		_CPU_AND_GPU_CODE_ inline friend bool operator == (const Matrix3 &lhs, const Matrix3 &rhs) {
			bool r = lhs.m[0] == rhs.m[0];
			for (int i = 1; i < 9; i++)
				r &= lhs.m[i] == rhs.m[i];
			return r;
		}

		_CPU_AND_GPU_CODE_ inline friend bool operator != (const Matrix3 &lhs, const Matrix3 &rhs) {
			bool r = lhs.m[0] != rhs.m[0];
			for (int i = 1; i < 9; i++)
				r |= lhs.m[i] != rhs.m[i];
			return r;
		}

		// Matrix determinant
		_CPU_AND_GPU_CODE_ inline T det() const {
			return (this->m11*this->m22 - this->m12*this->m21)*this->m00 + (this->m12*this->m20 - this->m10*this->m22)*this->m01 + (this->m10*this->m21 - this->m11*this->m20)*this->m02;
		}

		// The inverse matrix for float/double type
		_CPU_AND_GPU_CODE_ inline bool inv(Matrix3 &out) const {
			T determinant = det();
			if (determinant == 0) {
				out.setZeros();
				return false;
			}

			out.m00 = (this->m11*this->m22 - this->m12*this->m21) / determinant;
			out.m01 = (this->m02*this->m21 - this->m01*this->m22) / determinant;
			out.m02 = (this->m01*this->m12 - this->m02*this->m11) / determinant;
			out.m10 = (this->m12*this->m20 - this->m10*this->m22) / determinant;
			out.m11 = (this->m00*this->m22 - this->m02*this->m20) / determinant;
			out.m12 = (this->m02*this->m10 - this->m00*this->m12) / determinant;
			out.m20 = (this->m10*this->m21 - this->m11*this->m20) / determinant;
			out.m21 = (this->m01*this->m20 - this->m00*this->m21) / determinant;
			out.m22 = (this->m00*this->m11 - this->m01*this->m10) / determinant;
			return true;
		}

		friend std::ostream& operator<<(std::ostream& os, const Matrix3<T>& dt)	{
			for (int y = 0; y < 3; y++)
				os << dt(0, y) << ", " << dt(1, y) << ", " << dt(2, y) << "\n";
			return os;
		}
	};

	template<class T, int s>
	class MatrixSQX : public MatrixSQX_ < T, s >
	{
	public:
		_CPU_AND_GPU_CODE_ MatrixSQX() { this->dim = s; this->sq = s*s; }
		_CPU_AND_GPU_CODE_ MatrixSQX(T t) { this->dim = s; this->sq = s*s; setValues(t); }
		_CPU_AND_GPU_CODE_ MatrixSQX(const T *m)	{ this->dim = s; this->sq = s*s; setValues(m); }

		_CPU_AND_GPU_CODE_ inline void getValues(T *mp) const	{ memcpy(mp, this->m, sizeof(T) * 16); }
		_CPU_AND_GPU_CODE_ inline const T *getValues() const { return this->m; }

		// Element access
		_CPU_AND_GPU_CODE_ inline T &operator()(int x, int y)	{ return at(x, y); }
		_CPU_AND_GPU_CODE_ inline const T &operator()(int x, int y) const	{ return at(x, y); }
		_CPU_AND_GPU_CODE_ inline T &operator()(Vector2<int> pnt)	{ return at(pnt.x, pnt.y); }
		_CPU_AND_GPU_CODE_ inline const T &operator()(Vector2<int> pnt) const	{ return at(pnt.x, pnt.y); }
		_CPU_AND_GPU_CODE_ inline T &at(int x, int y) { return this->m[y * s + x]; }
		_CPU_AND_GPU_CODE_ inline const T &at(int x, int y) const { return this->m[y * s + x]; }

		// set values
		_CPU_AND_GPU_CODE_ inline void setValues(const T *mp) { for (int i = 0; i < s*s; i++) this->m[i] = mp[i]; }
		_CPU_AND_GPU_CODE_ inline void setValues(T r)	{ for (int i = 0; i < s*s; i++)	this->m[i] = r; }
		_CPU_AND_GPU_CODE_ inline void setZeros() { for (int i = 0; i < s*s; i++)	this->m[i] = 0; }
		_CPU_AND_GPU_CODE_ inline void setIdentity() { setZeros(); for (int i = 0; i < s*s; i++) this->m[i + i*s] = 1; }

		// get values
		_CPU_AND_GPU_CODE_ inline VectorX<T, s> getRow(int r) const { VectorX<T, s> v; for (int x = 0; x < s; x++) v[x] = at(x, r); return v; }
		_CPU_AND_GPU_CODE_ inline VectorX<T, s> getColumn(int c) const { Vector4<T> v; for (int x = 0; x < s; x++) v[x] = at(c, x); return v; }
		_CPU_AND_GPU_CODE_ inline MatrixSQX<T, s> getTranspose()
		{ // transpose
			MatrixSQX<T, s> mtrans;
			for (int x = 0; x < s; x++)	for (int y = 0; y < s; y++)
				mtrans(x, y) = at(y, x);
			return mtrans;
		}

		_CPU_AND_GPU_CODE_ inline friend  MatrixSQX<T, s> operator * (const  MatrixSQX<T, s> &lhs, const  MatrixSQX<T, s> &rhs)	{
			MatrixSQX<T, s> r;
			r.setZeros();
			for (int x = 0; x < s; x++) for (int y = 0; y < s; y++) for (int k = 0; k < s; k++)
				r(x, y) += lhs(k, y) * rhs(x, k);
			return r;
		}

		_CPU_AND_GPU_CODE_ inline friend MatrixSQX<T, s> operator + (const MatrixSQX<T, s> &lhs, const MatrixSQX<T, s> &rhs) {
			MatrixSQX<T, s> res(lhs.m);
			return res += rhs;
		}

		_CPU_AND_GPU_CODE_ inline MatrixSQX<T, s>& operator += (const T &r) { for (int i = 0; i < s*s; ++i) this->m[i] += r; return *this; }
		_CPU_AND_GPU_CODE_ inline MatrixSQX<T, s>& operator -= (const T &r) { for (int i = 0; i < s*s; ++i) this->m[i] -= r; return *this; }
		_CPU_AND_GPU_CODE_ inline MatrixSQX<T, s>& operator *= (const T &r) { for (int i = 0; i < s*s; ++i) this->m[i] *= r; return *this; }
		_CPU_AND_GPU_CODE_ inline MatrixSQX<T, s>& operator /= (const T &r) { for (int i = 0; i < s*s; ++i) this->m[i] /= r; return *this; }
		_CPU_AND_GPU_CODE_ inline MatrixSQX<T, s> &operator += (const MatrixSQX<T, s> &mat) { for (int i = 0; i < s*s; ++i) this->m[i] += mat.m[i]; return *this; }
		_CPU_AND_GPU_CODE_ inline MatrixSQX<T, s> &operator -= (const MatrixSQX<T, s> &mat) { for (int i = 0; i < s*s; ++i) this->m[i] -= mat.m[i]; return *this; }

		_CPU_AND_GPU_CODE_ inline friend bool operator == (const MatrixSQX<T, s> &lhs, const MatrixSQX<T, s> &rhs) {
			bool r = lhs.m[0] == rhs.m[0];
			for (int i = 1; i < s*s; i++)
				r &= lhs.m[i] == rhs.m[i];
			return r;
		}

		_CPU_AND_GPU_CODE_ inline friend bool operator != (const MatrixSQX<T, s> &lhs, const MatrixSQX<T, s> &rhs) {
			bool r = lhs.m[0] != rhs.m[0];
			for (int i = 1; i < s*s; i++)
				r |= lhs.m[i] != rhs.m[i];
			return r;
		}

		friend std::ostream& operator<<(std::ostream& os, const MatrixSQX<T, s>& dt) {
			for (int y = 0; y < s; y++)
			{
				for (int x = 0; x < s; x++) os << dt(x, y) << "\t";
				os << "\n";
			}
			return os;
		}
	};


}

