// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMMath.h"

namespace ITMLib
{
	/** \brief
	    Represents the parameters for projection with a projective
	    camera
	*/
	class ITMIntrinsics
	{
	public:
		/** The actual intrinsic calibration parameters. */
		struct ProjectionParamsSimple
		{
			Vector4f all;
			float fx, fy, px, py;
		} projectionParamsSimple;

		/** Setup all the internal members of this class from
		    the given parameters. Everything is in pixel
		    coordinates.
			@param fx Focal length in x direction
			@param fy Focal length in y direction
			@param cx Principal point in x direction
			@param cy Principal point in y direction
		*/
		void SetFrom(float fx, float fy, float cx, float cy)
		{
			projectionParamsSimple.fx = fx; 
			projectionParamsSimple.fy = fy;
			projectionParamsSimple.px = cx; 
			projectionParamsSimple.py = cy;

			projectionParamsSimple.all.x = fx; 
			projectionParamsSimple.all.y = fy;
			projectionParamsSimple.all.z = cx; 
			projectionParamsSimple.all.w = cy;
		}

		/**
		 * @brief Returns true if the two focal lengths have a different sign.
		 *
		 * @note  This is used to handle datasets such as ICL_NUIM and other non standard inputs
		 * 	      where one of the two focal lengths is negative: that causes the normals to point
		 * 	      away from the camera. This causes valid points to be ignored during visualisation
		 * 	      and tracking.
		 *
		 * 	      The problem presents itself only when computing normals as cross product of the
		 * 	      difference vectors between raycasted points and is thus solved by flipping
		 * 	      the normal direction.
		 */
		bool FocalLengthSignsDiffer() const
		{
			return projectionParamsSimple.fx * projectionParamsSimple.fy < 0.f;
		}

		ITMIntrinsics(void)
		{
			// standard calibration parameters for Kinect RGB camera. Not at all
			// accurate, though...
			SetFrom(580, 580, 320, 240);
		}
	};
}
