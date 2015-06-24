// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

namespace ITMLib
{
	namespace Objects
	{
		/** \brief
		    Represents a camera pose with rotation and translation
		    parameters
		*/
		class ITMPose
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
			Matrix4f M;

			/** This will update the minimal parameterisation from
			    the current modelview matrix.
			*/
			void SetParamsFromModelView();

			/** This will update the "modelview matrix" M from the
			    minimal representation.
			*/
			void SetModelViewFromParams();
		public:

			void SetFrom(float tx, float ty, float tz, float rx, float ry, float rz);
			void SetFrom(const Vector3f &translation, const Vector3f &rotation);
			void SetFrom(const Vector6f &tangent);

			void SetFrom(const float pose[6]);
			void SetFrom(const ITMPose *pose);

			/** This will multiply a pose @p pose on the right, i.e.
			    this = this * pose.
			*/
			void MultiplyWith(const ITMPose *pose);

			const Matrix4f & GetM(void) const
			{ return M; }

			Matrix3f GetR(void) const;
			Vector3f GetT(void) const;

			void GetParams(Vector3f &translation, Vector3f &rotation);

			void SetM(const Matrix4f & M);

			void SetR(const Matrix3f & R);
			void SetT(const Vector3f & t);
			void SetRT(const Matrix3f & R, const Vector3f & t);

			Matrix4f GetInvM(void) const;
			void SetInvM(const Matrix4f & invM);

			/** This will enforce the orthonormality constraints on
			    the rotation matrix. It's recommended to call this
			    function after manipulating the matrix M.
			*/
			void Coerce(void);

			ITMPose(const ITMPose & src);
			ITMPose(const Matrix4f & src);
			ITMPose(float tx, float ty, float tz, float rx, float ry, float rz);
			ITMPose(const Vector6f & tangent);
			explicit ITMPose(const float pose[6]);

			ITMPose(void);

			/** This builds a Pose based on its exp representation
			*/
			static ITMPose exp(const Vector6f& tangent);
		};
	}
}
