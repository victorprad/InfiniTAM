// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include <math.h>
#include "ITMPose.h"

#include <stdio.h>

using namespace ITMLib::Objects;

ITMPose::ITMPose(void) { this->SetFrom(0, 0, 0, 0, 0, 0); }

ITMPose::ITMPose(float tx, float ty, float tz, float rx, float ry, float rz) 
{ this->SetFrom(tx, ty, tz, rx, ry, rz); }
ITMPose::ITMPose(const float pose[6]) { this->SetFrom(pose); }
ITMPose::ITMPose(const Matrix4f & src) { this->SetFrom(src); }

#ifndef M_SQRT1_2
#define M_SQRT1_2 0.707106781186547524401
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.5707963267948966192E0
#endif

void ITMPose::SetFrom(float tx, float ty, float tz, float rx, float ry, float rz)
{
	this->params.each.tx = tx;
	this->params.each.ty = ty;
	this->params.each.tz = tz;
	this->params.each.rx = rx;
	this->params.each.ry = ry;
	this->params.each.rz = rz;

	this->SetModelViewFromParams();
}

void ITMPose::SetFrom(const float pose[6])
{
	this->params.each.tx = pose[0];
	this->params.each.ty = pose[1];
	this->params.each.tz = pose[2];
	this->params.each.rx = pose[3];
	this->params.each.ry = pose[4];
	this->params.each.rz = pose[5];
}

void ITMPose::SetFrom(const ITMPose *pose)
{
	this->params.each.tx = pose->params.each.tx;
	this->params.each.ty = pose->params.each.ty;
	this->params.each.tz = pose->params.each.tz;
	this->params.each.rx = pose->params.each.rx;
	this->params.each.ry = pose->params.each.ry;
	this->params.each.rz = pose->params.each.rz;

	//SetMatrixFromMatrix_4(&this->M, &pose->M);
	//SetMatrixFromMatrix_4(&this->invM, &pose->invM);
	M = pose->M;
	invM = pose->invM;

	//SetMatrixFromMatrix_3(&this->R, &pose->R);
	//SetMatrixFromMatrix_3(&this->invR, &pose->invR);
	R = pose->R;
	invR = pose->invR;
	
	//SetVectorFromVector_3(&this->T, &pose->T);
	//SetVectorFromVector_3(&this->invT, &pose->invT);
	T = pose->T;
	invT = pose->invT;
}

void ITMPose::SetFrom(const Matrix4f & src)
{
	M = src;
	for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) R.m[r+3*c] = src.m[r+4*c];
	for (int r = 0; r < 3; r++) T.v[r] = R.m[r+3*4];
	this->SetRTInvM_FromM();
	SetParamsFromModelView();
	SetModelViewFromParams();
}

void ITMPose::SetModelViewFromParams()
{
	float one_6th = 1.0f/6.0f;
	float one_20th = 1.0f/20.0f;

	Vector3f w; w.x = params.each.rx; w.y = params.each.ry; w.z = params.each.rz;
	Vector3f t; t.x = params.each.tx; t.y = params.each.ty; t.z = params.each.tz;

	//float theta_sq = VectorDotProduct_3(&w, &w);
	float theta_sq = dot(w, w);
	float theta = sqrtf(theta_sq);

	float A, B;

	Vector3f crossV = cross(w, t);// , buffV3; VectorCrossProduct_3(&crossV, &w, &t, &buffV3);
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

		Vector3f cross2 = cross(w, crossV);
		//VectorCrossProduct_3(&cross2, &w, &crossV, &buffV3);

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

	this->SetRTInvM_FromM();
}

void ITMPose::SetParamsFromModelView()
{
	Vector3f resultRot;
	
	float cos_angle = (R.m00  + R.m11 + R.m22 - 1.0f) * 0.5f;
	resultRot.x = (R.m[2 + 3 * 1] - R.m[1 + 3 * 2]) * 0.5f;
	resultRot.y = (R.m[0 + 3 * 2] - R.m[2 + 3 * 0]) * 0.5f;
	resultRot.z = (R.m[1 + 3 * 0] - R.m[0 + 3 * 1]) * 0.5f;

	//float sin_angle_abs = sqrtf(VectorDotProduct_3(&resultRot, &resultRot));
	float sin_angle_abs = sqrtf(dot(resultRot, resultRot));

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

			Vector3f r2;

			if(fabsf(d0) > fabsf(d1) && fabsf(d0) > fabsf(d2))
			{ r2.x = d0; r2.y = (R.m[1 + 3 * 0] + R.m[0 + 3 * 1]) * 0.5f; r2.z = (R.m[0 + 3 * 2] + R.m[2 + 3 * 0]) * 0.5f; } 
			else 
			{
				if(fabsf(d1) > fabsf(d2)) 
				{ r2.x = (R.m[1 + 3 * 0] + R.m[0 + 3 * 1]) * 0.5f; r2.y = d1; r2.z = (R.m[2 + 3 * 1] + R.m[1 + 3 * 2]) * 0.5f; }
				else { r2.x = (R.m[0 + 3 * 2] + R.m[2 + 3 * 0]) * 0.5f; r2.y = (R.m[2 + 3 * 1] + R.m[1 + 3 * 2]) * 0.5f; r2.z = d2; }
			}

			//if (VectorDotProduct_3(&r2, &resultRot) < 0.0f)
			if (dot(r2, resultRot) < 0.0f) { r2.x *= -1.0f; r2.y *= -1.0f; r2.z *= -1.0f; }

			//VectorNormalize_3(&r2, &r2);
			r2 = normalize(r2);

			resultRot.x = angle * r2.x; resultRot.y = angle * r2.y; resultRot.z = angle * r2.z;
		}
	}

	float shtot = 0.5f;
	//float theta = sqrtf(VectorDotProduct_3(&resultRot, &resultRot));
	float theta = sqrtf(dot(resultRot, resultRot));

	if (theta > 0.00001f) shtot = sinf(theta * 0.5f) / theta;

	ITMPose halfrotor;
	float halfrotorparams[6];
	halfrotorparams[0] = 0.0f; halfrotorparams[1] = 0.0f; halfrotorparams[2] = 0.0f;
	halfrotorparams[3] = resultRot.x * -0.5f; halfrotorparams[4] = resultRot.y * -0.5f; halfrotorparams[5] = resultRot.z * -0.5f; 
	halfrotor.SetFrom(halfrotorparams); halfrotor.SetModelViewFromParams();

	//Vector3f rottrans, buffV3;
	//MatrixVectorMultiply_3(&rottrans, &halfrotor.R, &this->T, &buffV3);
	Vector3f rottrans = halfrotor.R * T;

	if (theta > 0.001f)
	{
		//float denom = VectorDotProduct_3(&resultRot, &resultRot);
		//float param = VectorDotProduct_3(&this->T, &resultRot) * (1 - 2 * shtot) / denom;
		float denom = dot(resultRot, resultRot);
		float param = dot(this->T, resultRot) * (1 - 2 * shtot) / denom;
		
		rottrans.x -= resultRot.x * param; rottrans.y -= resultRot.y * param; rottrans.z -= resultRot.z * param;
	}
	else
	{
		//float param = VectorDotProduct_3(&this->T, &resultRot) / 24;
		float param = dot(this->T, resultRot) / 24;
		rottrans.x -= resultRot.x * param; rottrans.y -= resultRot.y * param; rottrans.z -= resultRot.z * param;
	}

	rottrans.x /= 2 * shtot; rottrans.y /= 2 * shtot; rottrans.z /= 2 * shtot;

	this->params.each.rx = resultRot.x; this->params.each.ry = resultRot.y; this->params.each.rz = resultRot.z;
	this->params.each.tx = rottrans.x; this->params.each.ty = rottrans.y; this->params.each.tz = rottrans.z; 
}

void ITMPose::MultiplyWith(const ITMPose *pose)
{
	Matrix3f m_out/*, buffM3*/; Vector3f t_out;
	//MatrixMultiply_3(&m_out, &this->R, &pose->R, &buffM3);
	m_out = R * pose->R;
	//VectorSum_3(&t_out, &this->T, &pose->T);
	t_out = T + pose->T;

	M.m[0 + 4 * 0] = m_out.m[0 + 3 * 0]; M.m[1 + 4 * 0] = m_out.m[1 + 3 * 0]; M.m[2 + 4 * 0] = m_out.m[2 + 3 * 0];
	M.m[0 + 4 * 1] = m_out.m[0 + 3 * 1]; M.m[1 + 4 * 1] = m_out.m[1 + 3 * 1]; M.m[2 + 4 * 1] = m_out.m[2 + 3 * 1];
	M.m[0 + 4 * 2] = m_out.m[0 + 3 * 2]; M.m[1 + 4 * 2] = m_out.m[1 + 3 * 2]; M.m[2 + 4 * 2] = m_out.m[2 + 3 * 2];
	M.m[0 + 4 * 3] = t_out.v[0]; M.m[1 + 4 * 3] = t_out.v[1]; M.m[2 + 4 * 3] = t_out.v[2];	

	this->SetRTInvM_FromM();
}

void ITMPose::SetRTInvM_FromM()
{
	R.m[0 + 3 * 0] = M.m[0 + 4 * 0]; R.m[1 + 3 * 0] = M.m[1 + 4 * 0]; R.m[2 + 3 * 0] = M.m[2 + 4 * 0];
	R.m[0 + 3 * 1] = M.m[0 + 4 * 1]; R.m[1 + 3 * 1] = M.m[1 + 4 * 1]; R.m[2 + 3 * 1] = M.m[2 + 4 * 1];
	R.m[0 + 3 * 2] = M.m[0 + 4 * 2]; R.m[1 + 3 * 2] = M.m[1 + 4 * 2]; R.m[2 + 3 * 2] = M.m[2 + 4 * 2];
	T.v[0] = M.m[0 + 4 * 3]; T.v[1] = M.m[1 + 4 * 3]; T.v[2] = M.m[2 + 4 * 3];

	//MatrixInvert_4(&invM, &M);
	M.inv(invM);

	invR.m[0 + 3 * 0] = invM.m[0 + 4 * 0]; invR.m[1 + 3 * 0] = invM.m[1 + 4 * 0]; invR.m[2 + 3 * 0] = invM.m[2 + 4 * 0];
	invR.m[0 + 3 * 1] = invM.m[0 + 4 * 1]; invR.m[1 + 3 * 1] = invM.m[1 + 4 * 1]; invR.m[2 + 3 * 1] = invM.m[2 + 4 * 1];
	invR.m[0 + 3 * 2] = invM.m[0 + 4 * 2]; invR.m[1 + 3 * 2] = invM.m[1 + 4 * 2]; invR.m[2 + 3 * 2] = invM.m[2 + 4 * 2];
	invT.v[0] = invM.m[0 + 4 * 3]; invT.v[1] = invM.m[1 + 4 * 3]; invT.v[2] = invM.m[2 + 4 * 3];
}

