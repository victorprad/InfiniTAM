// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include <stdlib.h>

namespace ITMLib
{
	namespace Objects
	{
		/** \brief
		    Represents the calibration information to compute a depth
		    image from the disparity image typically received from a
		    Kinect.
		*/
		class ITMDisparityCalib
		{
		public:
			/** Type of transformation required to get from raw
			    values to depths.
			*/
			typedef enum {
				/// Raw values are transformed according to \f$\frac{8c_2f_x}{c_1 - d}\f$
				TRAFO_KINECT,
				/// Raw values are transformed according to \f$c_1 d + c_2\f$
				TRAFO_AFFINE
			} TrafoType;

			TrafoType type;

			/** These are the actual parameters. */
			Vector2f params;

			/** Setup from given arguments. */
			void SetFrom(float a, float b, TrafoType _type)
			{ params.x = a; params.y = b; type = _type; }

			ITMDisparityCalib(void)
			{
				// standard calibration parameters - converts mm to metres by dividing by 1000
				params.x = 1.0f/1000.0f; params.y = 0.0f;
				type = TRAFO_AFFINE;
			}
		};
	}
}
