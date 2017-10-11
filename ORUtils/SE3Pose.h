// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "Vector.h"
#include "Matrix.h"

namespace ORUtils
{
	/** \brief
		Represents a camera pose with rotation and translation
		parameters
	*/
	class SE3Pose
	{
	private:
		/** This is the minimal representation of the pose with
			six parameters. The three rotation parameters are
			the Lie algebra representation of SO3.
		*/
		union
		{
			float all[6];
			struct {
				float tx, ty, tz;
				float rx, ry, rz;
			}each;
		} params;

		/** The pose as a 4x4 transformation matrix ("modelview
			matrix).
		*/
		Matrix4<float> M;

		/** This will update the minimal parameterisation from
			the current modelview matrix.
			*/
		void SetParamsFromModelView();

		/** This will update the "modelview matrix" M from the
			minimal representation.
		*/
		void SetModelViewFromParams();
	public:

		void SetBoth(const Matrix4<float> & M, const float params[6]);

		void SetFrom(float tx, float ty, float tz, float rx, float ry, float rz);
		void SetFrom(const Vector3<float> &translation, const Vector3<float> &rotation);
		void SetFrom(const Vector6<float> &tangent);

		void SetFrom(const float pose[6]);
		void SetFrom(const SE3Pose *pose);

		/** This will multiply a pose @p pose on the right, i.e.
			this = this * pose.
		*/
		void MultiplyWith(const SE3Pose *pose);

		const Matrix4<float> & GetM(void) const { return M; }

		Matrix3<float> GetR(void) const;
		Vector3<float> GetT(void) const;

		void GetParams(Vector3<float> &translation, Vector3<float> &rotation) const;
		const float* GetParams(void) const { return params.all; }

		void SetM(const Matrix4<float> & M);

		void SetR(const Matrix3<float> & R);
		void SetT(const Vector3<float> & t);
		void SetRT(const Matrix3<float> & R, const Vector3<float> & t);

		Matrix4<float> GetInvM(void) const;
		void SetInvM(const Matrix4<float> & invM);

		/** This will enforce the orthonormality constraints on
			the rotation matrix. It's recommended to call this
			function after manipulating the matrix M.
		*/
		void Coerce(void);

		SE3Pose(const SE3Pose & src);
		SE3Pose(const Matrix4<float> & src);
		SE3Pose(float tx, float ty, float tz, float rx, float ry, float rz);
		SE3Pose(const Vector6<float> & tangent);
		SE3Pose(const Matrix3<float> &R, const Vector3<float> &t);
		explicit SE3Pose(const float pose[6]);

		SE3Pose(void);

		friend std::ostream& operator<<(std::ostream& os, const SE3Pose& dt) {
			os << dt.M;
			return os;
		}

		/** This builds a Pose based on its exp representation
		*/
		static SE3Pose exp(const Vector6<float>& tangent);
	};
}

