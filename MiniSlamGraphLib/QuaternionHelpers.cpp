// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "QuaternionHelpers.h"

#include <math.h>

using namespace MiniSlamGraph;

static int QuaternionFromRotationMatrix_variant(const double *matrix)
{
	int variant = 0;
	if ((matrix[4] > -matrix[8]) && (matrix[0] > -matrix[4]) && (matrix[0] > -matrix[8])) {
		variant = 0;
	}
	else if ((matrix[4] < -matrix[8]) && (matrix[0] > matrix[4]) && (matrix[0] > matrix[8])) {
		variant = 1;
	}
	else if ((matrix[4] > matrix[8]) && (matrix[0] < matrix[4]) && (matrix[0] < -matrix[8])) {
		variant = 2;
	}
	else if ((matrix[4] < matrix[8]) && (matrix[0] < -matrix[4]) && (matrix[0] < matrix[8])) {
		variant = 3;
	}
	return variant;
}

static int QuaternionFromRotationMatrix_switchSign(const double *matrix, int variant = -1)
{
	if (variant == -1) variant = QuaternionFromRotationMatrix_variant(matrix);
	if (variant == 0) {
		return (1.0f + matrix[0] + matrix[4] + matrix[8]) < 0.0f;
	}
	else if (variant == 1) {
		return (matrix[5] - matrix[7]) < 0.0f;
	}
	else if (variant == 2) {
		return (matrix[6] - matrix[2]) < 0.0f;
	}
	else if (variant == 3) {
		return (matrix[1] - matrix[3]) < 0.0f;
	}
	return false;
}

void QuaternionHelpers::RotationMatrixFromQuaternion(const double *q, double *matrix)
{
	matrix[0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
	matrix[4] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
	matrix[8] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
	matrix[3] = 2 * (q[1] * q[2] - q[0] * q[3]);
	matrix[1] = 2 * (q[1] * q[2] + q[0] * q[3]);
	matrix[6] = 2 * (q[1] * q[3] + q[0] * q[2]);
	matrix[2] = 2 * (q[1] * q[3] - q[0] * q[2]);
	matrix[7] = 2 * (q[2] * q[3] - q[0] * q[1]);
	matrix[5] = 2 * (q[2] * q[3] + q[0] * q[1]);
}

void QuaternionHelpers::QuaternionFromRotationMatrix(const double *matrix, double *q)
{
	/* taken from "James Diebel. Representing Attitude: Euler Angles,
	   Quaternions, and Rotation Vectors. Technical Report, Stanford
	   University, Palo Alto, CA."
	*/

	// choose the numerically best variant...
	int variant = QuaternionFromRotationMatrix_variant(matrix);
	double denom = 1.0;
	if (variant == 0) {
		denom += matrix[0] + matrix[4] + matrix[8];
	}
	else {
		int tmp = variant * 4;
		denom += matrix[tmp - 4];
		denom -= matrix[tmp % 12];
		denom -= matrix[(tmp + 4) % 12];
	}
	denom = sqrt(denom);
	q[variant] = 0.5*denom;

	denom *= 2.0;
	switch (variant) {
	case 0:
		q[1] = (matrix[5] - matrix[7]) / denom;
		q[2] = (matrix[6] - matrix[2]) / denom;
		q[3] = (matrix[1] - matrix[3]) / denom;
		break;
	case 1:
		q[0] = (matrix[5] - matrix[7]) / denom;
		q[2] = (matrix[1] + matrix[3]) / denom;
		q[3] = (matrix[6] + matrix[2]) / denom;
		break;
	case 2:
		q[0] = (matrix[6] - matrix[2]) / denom;
		q[1] = (matrix[1] + matrix[3]) / denom;
		q[3] = (matrix[5] + matrix[7]) / denom;
		break;
	case 3:
		q[0] = (matrix[1] - matrix[3]) / denom;
		q[1] = (matrix[6] + matrix[2]) / denom;
		q[2] = (matrix[5] + matrix[7]) / denom;
		break;
	}

	if (q[0] < 0.0f) for (int i = 0; i < 4; ++i) q[i] *= -1.0f;
}

void QuaternionHelpers::dQuaternion_dRotationMatrix(const double *matrix, double *dq_dR)
{
	/* basic conversion taken from "James Diebel. Representing Attitude:
	   Euler Angles, Quaternions, and Rotation Vectors. Technical Report,
	   Stanford University, Palo Alto, CA."
	   Derivatives of the basic conversions are "trivial" to compute... :)
	*/
	int variant = QuaternionFromRotationMatrix_variant(matrix);

	const double & r11 = matrix[0];
	const double & r12 = matrix[1];
	const double & r13 = matrix[2];
	const double & r21 = matrix[3];
	const double & r22 = matrix[4];
	const double & r23 = matrix[5];
	const double & r31 = matrix[6];
	const double & r32 = matrix[7];
	const double & r33 = matrix[8];

	double denom = 0.0;
	if (variant == 0) {
		denom = r11 + r22 + r33 + 1.0f;
	}
	else if (variant == 1) {
		denom = r11 - r22 - r33 + 1.0f;
	}
	else if (variant == 2) {
		denom = -r11 + r22 - r33 + 1.0f;
	}
	else if (variant == 3) {
		denom = -r11 - r22 + r33 + 1.0f;
	}
	double sqrt_denom = sqrt(denom);
	double d_sqrt_denom_dR11, d_sqrt_denom_dR22, d_sqrt_denom_dR33;
	double part32_sign = 1.0f;
	double part13_sign = 1.0f;
	double part21_sign = 1.0f;
	int denom_index, part32_index, part13_index, part21_index;
	if (variant == 0) {
		d_sqrt_denom_dR11 = 0.5f / sqrt_denom;
		d_sqrt_denom_dR22 = 0.5f / sqrt_denom;
		d_sqrt_denom_dR33 = 0.5f / sqrt_denom;
		part32_sign = -1.0f;
		part13_sign = -1.0f;
		part21_sign = -1.0f;
		denom_index = 0;
		part32_index = 1;
		part13_index = 2;
		part21_index = 3;
	}
	else if (variant == 1) {
		d_sqrt_denom_dR11 = 0.5f / sqrt_denom;
		d_sqrt_denom_dR22 = -0.5f / sqrt_denom;
		d_sqrt_denom_dR33 = -0.5f / sqrt_denom;
		part32_sign = -1.0f;
		part13_sign = 1.0f;
		part21_sign = 1.0f;
		denom_index = 1;
		part32_index = 0;
		part13_index = 3;
		part21_index = 2;
	}
	else if (variant == 2) {
		d_sqrt_denom_dR11 = -0.5f / sqrt_denom;
		d_sqrt_denom_dR22 = 0.5f / sqrt_denom;
		d_sqrt_denom_dR33 = -0.5f / sqrt_denom;
		part32_sign = 1.0f;
		part13_sign = -1.0f;
		part21_sign = 1.0f;
		denom_index = 2;
		part32_index = 3;
		part13_index = 0;
		part21_index = 1;
	}
	else if (variant == 3) {
		d_sqrt_denom_dR11 = -0.5f / sqrt_denom;
		d_sqrt_denom_dR22 = -0.5f / sqrt_denom;
		d_sqrt_denom_dR33 = 0.5f / sqrt_denom;
		part32_sign = 1.0f;
		part13_sign = 1.0f;
		part21_sign = -1.0f;
		denom_index = 3;
		part32_index = 2;
		part13_index = 1;
		part21_index = 0;
	}

	for (int i = 0; i < 9 * 4; ++i) dq_dR[i] = 0.0f;

	// "simple" line with only denominator
	dq_dR[denom_index * 9 + 0] = d_sqrt_denom_dR11;
	dq_dR[denom_index * 9 + 4] = d_sqrt_denom_dR22;
	dq_dR[denom_index * 9 + 8] = d_sqrt_denom_dR33;

	// line with the r23 +- r32 part
	dq_dR[part32_index * 9 + 5] = 1.0f / sqrt_denom;
	dq_dR[part32_index * 9 + 7] = part32_sign / sqrt_denom;
	dq_dR[part32_index * 9 + 0] = -(r23 + part32_sign*r32)*d_sqrt_denom_dR11 / denom;
	dq_dR[part32_index * 9 + 4] = -(r23 + part32_sign*r32)*d_sqrt_denom_dR22 / denom;
	dq_dR[part32_index * 9 + 8] = -(r23 + part32_sign*r32)*d_sqrt_denom_dR33 / denom;

	// line with the r31 +- r13 part
	dq_dR[part13_index * 9 + 6] = 1.0f / sqrt_denom;
	dq_dR[part13_index * 9 + 2] = part13_sign / sqrt_denom;
	dq_dR[part13_index * 9 + 0] = -(r31 + part13_sign*r13)*d_sqrt_denom_dR11 / denom;
	dq_dR[part13_index * 9 + 4] = -(r31 + part13_sign*r13)*d_sqrt_denom_dR22 / denom;
	dq_dR[part13_index * 9 + 8] = -(r31 + part13_sign*r13)*d_sqrt_denom_dR33 / denom;

	// line with the r12 +- r21 part
	dq_dR[part21_index * 9 + 1] = 1.0f / sqrt_denom;
	dq_dR[part21_index * 9 + 3] = part21_sign / sqrt_denom;
	dq_dR[part21_index * 9 + 0] = -(r12 + part21_sign*r21)*d_sqrt_denom_dR11 / denom;
	dq_dR[part21_index * 9 + 4] = -(r12 + part21_sign*r21)*d_sqrt_denom_dR22 / denom;
	dq_dR[part21_index * 9 + 8] = -(r12 + part21_sign*r21)*d_sqrt_denom_dR33 / denom;

	double factor = QuaternionFromRotationMatrix_switchSign(matrix, variant) ? -0.5f : 0.5f;
	for (int i = 0; i < 9 * 4; ++i) dq_dR[i] *= factor;
}
