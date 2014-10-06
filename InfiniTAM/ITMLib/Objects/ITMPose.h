// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

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
		public:
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
			}params;

			/** The pose as a 4x4 transformation matrix ("modelview
			    matrix).
			*/
			Matrix4f M;
			Matrix3f R; Vector3f T;

			/** The inverse of the pose as a 4x4 transformation
			    matrix.
			*/
			Matrix4f invM;
			Matrix3f invR; Vector3f invT;

			void SetFrom(float tx, float ty, float tz, float rx, float ry, float rz);
			void SetFrom(const float pose[6]);
			void SetFrom(const ITMPose *pose);
			void SetFrom(const Matrix4f & M);

			/** This will multiply a pose @p pose on the right, i.e.
			    this = this * pose.
			*/
			void MultiplyWith(const ITMPose *pose);

			/** This will update the minimal parameterisation from
			    the current modelview matrix.
			*/
			void SetParamsFromModelView();

			/** This will update the inverse modelview matrix */
			void SetRTInvM_FromM();

			/** This will update the "modelview matrix" M from the
			    minimal representation.
			*/
			void SetModelViewFromParams();

			ITMPose(const Matrix4f & src);
			ITMPose(float tx, float ty, float tz, float rx, float ry, float rz);
			explicit ITMPose(const float pose[6]);

			ITMPose(void);
		};
	}
}
