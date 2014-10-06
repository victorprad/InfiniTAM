// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

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
			void SetFrom(float fx, float fy, float cx, float cy, float sizeX, float sizeY);

			ITMIntrinsics(void);
		};
	}
}
