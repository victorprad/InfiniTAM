// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMMath.h"

namespace ITMLib
{
	/** \brief
	    Represents the extrinsic calibration between RGB and depth
	    cameras
	*/
	class ITMExtrinsics
	{
	public:
		/** The transformation matrix representing the
		    extrinsic calibration data.
		*/
		Matrix4f calib;
		/** Inverse of the above. */
		Matrix4f calib_inv;

		/** Setup from a given 4x4 matrix, where only the upper
		    three rows are used. More specifically, m00...m22
		    are expected to contain a rotation and m30...m32
		    contain the translation.
		*/
		void SetFrom(const Matrix4f & src)
		{
			this->calib = src;
			this->calib_inv.setIdentity();
			for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) this->calib_inv.m[r+4*c] = this->calib.m[c+4*r];
			for (int r = 0; r < 3; ++r) {
				float & dest = this->calib_inv.m[r+4*3];
				dest = 0.0f;
				for (int c = 0; c < 3; ++c) dest -= this->calib.m[c+4*r] * this->calib.m[c+4*3];
			}
		}

		ITMExtrinsics()
		{
			Matrix4f m;
			m.setZeros();
			m.m00 = m.m11 = m.m22 = m.m33 = 1.0;
			SetFrom(m);
		}
	};
}
