// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include <math.h>
#include "SE3Pose.h"

using namespace ORUtils;

SE3Pose::SE3Pose(void) { this->SetFrom(0, 0, 0, 0, 0, 0); }

SE3Pose::SE3Pose(float tx, float ty, float tz, float rx, float ry, float rz)
{
	this->SetFrom(tx, ty, tz, rx, ry, rz);
}
SE3Pose::SE3Pose(const float pose[6]) { this->SetFrom(pose); }
SE3Pose::SE3Pose(const Matrix4<float> & src) { this->SetM(src); }
SE3Pose::SE3Pose(const Vector6<float> & tangent) { this->SetFrom(tangent); }
SE3Pose::SE3Pose(const SE3Pose & src) { this->SetFrom(&src); }
SE3Pose::SE3Pose(const Matrix3<float> &R, const Vector3<float> &t) { this->SetRT(R, t); }

#ifndef M_SQRT1_2
#define M_SQRT1_2 0.707106781186547524401
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.5707963267948966192E0
#endif

void SE3Pose::SetBoth(const Matrix4<float> & M, const float params[6])
{
	this->M = M;
	memcpy(this->params.all, params, 6 * sizeof(float));
}

void SE3Pose::SetFrom(float tx, float ty, float tz, float rx, float ry, float rz)
{
	this->params.each.tx = tx;
	this->params.each.ty = ty;
	this->params.each.tz = tz;
	this->params.each.rx = rx;
	this->params.each.ry = ry;
	this->params.each.rz = rz;

	this->SetModelViewFromParams();
}

void SE3Pose::SetFrom(const Vector3<float> &translation, const Vector3<float> &rotation)
{
	this->params.each.tx = translation.x;
	this->params.each.ty = translation.y;
	this->params.each.tz = translation.z;
	this->params.each.rx = rotation.x;
	this->params.each.ry = rotation.y;
	this->params.each.rz = rotation.z;

	this->SetModelViewFromParams();
}

void SE3Pose::SetFrom(const Vector6<float> &tangent)
{
	this->params.each.tx = tangent[0];
	this->params.each.ty = tangent[1];
	this->params.each.tz = tangent[2];
	this->params.each.rx = tangent[3];
	this->params.each.ry = tangent[4];
	this->params.each.rz = tangent[5];

	this->SetModelViewFromParams();
}

void SE3Pose::SetFrom(const float pose[6])
{
	SetFrom(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
}

void SE3Pose::SetFrom(const SE3Pose *pose)
{
	this->params.each.tx = pose->params.each.tx;
	this->params.each.ty = pose->params.each.ty;
	this->params.each.tz = pose->params.each.tz;
	this->params.each.rx = pose->params.each.rx;
	this->params.each.ry = pose->params.each.ry;
	this->params.each.rz = pose->params.each.rz;

	M = pose->M;
}

void SE3Pose::SetModelViewFromParams()
{
	float one_6th = 1.0f / 6.0f;
	float one_20th = 1.0f / 20.0f;

	Vector3<float> w; w.x = params.each.rx; w.y = params.each.ry; w.z = params.each.rz;
	Vector3<float> t; t.x = params.each.tx; t.y = params.each.ty; t.z = params.each.tz;

	float theta_sq = dot(w, w);
	float theta = sqrt(theta_sq);

	float A, B;

	Matrix3<float> R; Vector3<float> T;

	Vector3<float> crossV = cross(w, t);
	if (theta_sq < 1e-8f)
	{
		A = 1.0f - one_6th * theta_sq; B = 0.5f;
		T.x = t.x + 0.5f * crossV.x; T.y = t.y + 0.5f * crossV.y; T.z = t.z + 0.5f * crossV.z;
	}
	else
	{
		float C;
		if (theta_sq < 1e-6f)
		{
			C = one_6th * (1.0f - one_20th * theta_sq);
			A = 1.0f - theta_sq * C;
			B = 0.5f - 0.25f * one_6th * theta_sq;
		}
		else
		{
			float inv_theta = 1.0f / theta;
			A = sinf(theta) * inv_theta;
			B = (1.0f - cosf(theta)) * (inv_theta * inv_theta);
			C = (1.0f - A) * (inv_theta * inv_theta);
		}

		Vector3<float> cross2 = cross(w, crossV);

		T.x = t.x + B * crossV.x + C * cross2.x; T.y = t.y + B * crossV.y + C * cross2.y; T.z = t.z + B * crossV.z + C * cross2.z;
	}

	float wx2 = w.x * w.x, wy2 = w.y * w.y, wz2 = w.z * w.z;
	R.m[0 + 3 * 0] = 1.0f - B*(wy2 + wz2);
	R.m[1 + 3 * 1] = 1.0f - B*(wx2 + wz2);
	R.m[2 + 3 * 2] = 1.0f - B*(wx2 + wy2);

	float a, b;
	a = A * w.z, b = B * (w.x * w.y);
	R.m[0 + 3 * 1] = b - a;
	R.m[1 + 3 * 0] = b + a;

	a = A * w.y, b = B * (w.x * w.z);
	R.m[0 + 3 * 2] = b + a;
	R.m[2 + 3 * 0] = b - a;

	a = A * w.x, b = B * (w.y * w.z);
	R.m[1 + 3 * 2] = b - a;
	R.m[2 + 3 * 1] = b + a;

	M.m[0 + 4 * 0] = R.m[0 + 3 * 0]; M.m[1 + 4 * 0] = R.m[1 + 3 * 0]; M.m[2 + 4 * 0] = R.m[2 + 3 * 0];
	M.m[0 + 4 * 1] = R.m[0 + 3 * 1]; M.m[1 + 4 * 1] = R.m[1 + 3 * 1]; M.m[2 + 4 * 1] = R.m[2 + 3 * 1];
	M.m[0 + 4 * 2] = R.m[0 + 3 * 2]; M.m[1 + 4 * 2] = R.m[1 + 3 * 2]; M.m[2 + 4 * 2] = R.m[2 + 3 * 2];

	M.m[0 + 4 * 3] = T.v[0]; M.m[1 + 4 * 3] = T.v[1]; M.m[2 + 4 * 3] = T.v[2];

	M.m[3 + 4 * 0] = 0.0f; M.m[3 + 4 * 1] = 0.0f; M.m[3 + 4 * 2] = 0.0f; M.m[3 + 4 * 3] = 1.0f;
}

void SE3Pose::SetParamsFromModelView()
{
	Vector3<float> resultRot;
	Matrix3<float> R = GetR();
	Vector3<float> T = GetT();

	float cos_angle = (R.m00 + R.m11 + R.m22 - 1.0f) * 0.5f;
	resultRot.x = (R.m[2 + 3 * 1] - R.m[1 + 3 * 2]) * 0.5f;
	resultRot.y = (R.m[0 + 3 * 2] - R.m[2 + 3 * 0]) * 0.5f;
	resultRot.z = (R.m[1 + 3 * 0] - R.m[0 + 3 * 1]) * 0.5f;

	float sin_angle_abs = sqrt(dot(resultRot, resultRot));

	if (cos_angle > M_SQRT1_2)
	{
		if (sin_angle_abs)
		{
			float p = asinf(sin_angle_abs) / sin_angle_abs;
			resultRot.x *= p; resultRot.y *= p; resultRot.z *= p;
		}
	}
	else
	{
		if (cos_angle > -M_SQRT1_2)
		{
			float p = acosf(cos_angle) / sin_angle_abs;
			resultRot.x *= p; resultRot.y *= p; resultRot.z *= p;
		}
		else
		{
			float angle = (float)M_PI - asinf(sin_angle_abs);
			float d0 = R.m[0 + 3 * 0] - cos_angle;
			float d1 = R.m[1 + 3 * 1] - cos_angle;
			float d2 = R.m[2 + 3 * 2] - cos_angle;

			Vector3<float> r2;

			if (fabsf(d0) > fabsf(d1) && fabsf(d0) > fabsf(d2))
			{
				r2.x = d0; r2.y = (R.m[1 + 3 * 0] + R.m[0 + 3 * 1]) * 0.5f; r2.z = (R.m[0 + 3 * 2] + R.m[2 + 3 * 0]) * 0.5f;
			}
			else
			{
				if (fabsf(d1) > fabsf(d2))
				{
					r2.x = (R.m[1 + 3 * 0] + R.m[0 + 3 * 1]) * 0.5f; r2.y = d1; r2.z = (R.m[2 + 3 * 1] + R.m[1 + 3 * 2]) * 0.5f;
				}
				else { r2.x = (R.m[0 + 3 * 2] + R.m[2 + 3 * 0]) * 0.5f; r2.y = (R.m[2 + 3 * 1] + R.m[1 + 3 * 2]) * 0.5f; r2.z = d2; }
			}

			if (dot(r2, resultRot) < 0.0f) { r2.x *= -1.0f; r2.y *= -1.0f; r2.z *= -1.0f; }

			r2 = normalize(r2);

			resultRot.x = angle * r2.x; resultRot.y = angle * r2.y; resultRot.z = angle * r2.z;
		}
	}

	float shtot = 0.5f;
	float theta = sqrt(dot(resultRot, resultRot));

	if (theta > 0.00001f) shtot = sinf(theta * 0.5f) / theta;

	SE3Pose halfrotor(0.0f, 0.0f, 0.0f, resultRot.x * -0.5f, resultRot.y * -0.5f, resultRot.z * -0.5f);

	Vector3<float> rottrans = halfrotor.GetR() * T;

	if (theta > 0.001f)
	{
		float denom = dot(resultRot, resultRot);
		float param = dot(T, resultRot) * (1 - 2 * shtot) / denom;

		rottrans.x -= resultRot.x * param; rottrans.y -= resultRot.y * param; rottrans.z -= resultRot.z * param;
	}
	else
	{
		float param = dot(T, resultRot) / 24;
		rottrans.x -= resultRot.x * param; rottrans.y -= resultRot.y * param; rottrans.z -= resultRot.z * param;
	}

	rottrans.x /= 2 * shtot; rottrans.y /= 2 * shtot; rottrans.z /= 2 * shtot;

	this->params.each.rx = resultRot.x; this->params.each.ry = resultRot.y; this->params.each.rz = resultRot.z;
	this->params.each.tx = rottrans.x; this->params.each.ty = rottrans.y; this->params.each.tz = rottrans.z;
}

SE3Pose SE3Pose::exp(const Vector6<float>& tangent)
{
	return SE3Pose(tangent);
}

void SE3Pose::MultiplyWith(const SE3Pose *pose)
{
	M = M * pose->M;
	this->SetParamsFromModelView();
}

Matrix3<float> SE3Pose::GetR(void) const
{
	Matrix3<float> R;
	R.m[0 + 3 * 0] = M.m[0 + 4 * 0]; R.m[1 + 3 * 0] = M.m[1 + 4 * 0]; R.m[2 + 3 * 0] = M.m[2 + 4 * 0];
	R.m[0 + 3 * 1] = M.m[0 + 4 * 1]; R.m[1 + 3 * 1] = M.m[1 + 4 * 1]; R.m[2 + 3 * 1] = M.m[2 + 4 * 1];
	R.m[0 + 3 * 2] = M.m[0 + 4 * 2]; R.m[1 + 3 * 2] = M.m[1 + 4 * 2]; R.m[2 + 3 * 2] = M.m[2 + 4 * 2];

	return R;
}

Vector3<float> SE3Pose::GetT(void) const
{
	Vector3<float> T;
	T.v[0] = M.m[0 + 4 * 3]; T.v[1] = M.m[1 + 4 * 3]; T.v[2] = M.m[2 + 4 * 3];

	return T;
}

void SE3Pose::GetParams(Vector3<float> &translation, Vector3<float> &rotation) const
{
	translation.x = this->params.each.tx;
	translation.y = this->params.each.ty;
	translation.z = this->params.each.tz;

	rotation.x = this->params.each.rx;
	rotation.y = this->params.each.ry;
	rotation.z = this->params.each.rz;
}

void SE3Pose::SetM(const Matrix4<float> & src)
{
	M = src;
	SetParamsFromModelView();
}

void SE3Pose::SetR(const Matrix3<float> & R)
{
	M.m[0 + 4 * 0] = R.m[0 + 3 * 0]; M.m[1 + 4 * 0] = R.m[1 + 3 * 0]; M.m[2 + 4 * 0] = R.m[2 + 3 * 0];
	M.m[0 + 4 * 1] = R.m[0 + 3 * 1]; M.m[1 + 4 * 1] = R.m[1 + 3 * 1]; M.m[2 + 4 * 1] = R.m[2 + 3 * 1];
	M.m[0 + 4 * 2] = R.m[0 + 3 * 2]; M.m[1 + 4 * 2] = R.m[1 + 3 * 2]; M.m[2 + 4 * 2] = R.m[2 + 3 * 2];

	SetParamsFromModelView();
}

void SE3Pose::SetT(const Vector3<float> & t)
{
	M.m[0 + 4 * 3] = t.v[0]; M.m[1 + 4 * 3] = t.v[1]; M.m[2 + 4 * 3] = t.v[2];

	SetParamsFromModelView();
}

void SE3Pose::SetRT(const Matrix3<float> & R, const Vector3<float> & t)
{
	M.m[0 + 4 * 0] = R.m[0 + 3 * 0]; M.m[1 + 4 * 0] = R.m[1 + 3 * 0]; M.m[2 + 4 * 0] = R.m[2 + 3 * 0];
	M.m[0 + 4 * 1] = R.m[0 + 3 * 1]; M.m[1 + 4 * 1] = R.m[1 + 3 * 1]; M.m[2 + 4 * 1] = R.m[2 + 3 * 1];
	M.m[0 + 4 * 2] = R.m[0 + 3 * 2]; M.m[1 + 4 * 2] = R.m[1 + 3 * 2]; M.m[2 + 4 * 2] = R.m[2 + 3 * 2];

	M.m[0 + 4 * 3] = t.v[0]; M.m[1 + 4 * 3] = t.v[1]; M.m[2 + 4 * 3] = t.v[2];

	SetParamsFromModelView();
}

Matrix4<float> SE3Pose::GetInvM(void) const
{
	Matrix4<float> ret;
	M.inv(ret);
	return ret;
}

void SE3Pose::SetInvM(const Matrix4<float> & invM)
{
	invM.inv(M);
	SetParamsFromModelView();
}

void SE3Pose::Coerce(void)
{
	SetParamsFromModelView();
	SetModelViewFromParams();
}