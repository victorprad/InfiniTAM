// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <string>

#include "../Utils/ITMLibDefines.h"

namespace ITMLib
{
	namespace Objects
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
				@param sizeX Image size in x direction
				@param sizeY Image size in y direction
			*/
			void SetFrom(float fx, float fy, float cx, float cy, float sizeX, float sizeY)
			{
				projectionParamsSimple.fx = fx; projectionParamsSimple.fy = fy;
				projectionParamsSimple.px = cx; projectionParamsSimple.py = cy;
				projectionParamsSimple.all.x = fx; projectionParamsSimple.all.y = fy;
				projectionParamsSimple.all.z = cx; projectionParamsSimple.all.w = cy;
			}

			ITMIntrinsics(void)
			{
				// standard calibration parameters for Kinect RGB camera. Not at all
				// accurate, though...
				SetFrom(580, 580, 320, 240, 640, 480);
			}
		};
	}
}
