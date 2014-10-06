// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

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
			/** These are the actual parameters. */
			Vector2f params;

			/** Setup from given arguments. */
			void SetFrom(float a, float b)
			{ params.x = a; params.y = b; }

			ITMDisparityCalib(void);
		};
	}
}
