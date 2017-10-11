// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

namespace MiniSlamGraph
{
	class QuaternionHelpers
	{
	public:
		static void RotationMatrixFromQuaternion(const double *q, double *matrix);

		/** Read a row-major 3x3 rotation matrix from @p matrix , compute the
			corresponding quaternion and store it in the 4-vector @p q.
			The first element of @p q will be real part, followed by the three
			imaginary parts.
		*/
		static void QuaternionFromRotationMatrix(const double *matrix, double *q);

		/** Compute the derivative of the transformation in QuaternionFromRotationMatrix()
			w.r.t. the elements of the rotation matrix @p matrix and write them
			to @p dq_dR. The first 9 elements of @p dq_dR will be the
			derivatives of the real part of the quaternion w.r.t. R11, R12, ...,
			followed by three similar rows for the imaginary parts.
		*/
		static void dQuaternion_dRotationMatrix(const double *matrix, double *dq_dR);
	};
}

