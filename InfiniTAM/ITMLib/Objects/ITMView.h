// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Objects/ITMRGBDCalib.h"
#include "../Utils/ITMCalibIO.h"

namespace ITMLib
{
	namespace Objects
	{
		/** \brief
		    Represents a single "view", i.e. RGB and depth images along
		    with all intrinsic and relative calibration information
		*/
		class ITMView
		{
		public:
			bool isAllocated;

			/// Intrinsic calibration information for the view.
			ITMRGBDCalib *calib;

			/// RGB colour image.
			ITMUChar4Image *rgb; 

			/// Float valued depth image, if available according to @ref inputImageType.
			ITMFloatImage *depth;

			ITMView() { isAllocated = false; }

			~ITMView(void)
			{
				if (isAllocated)
				{
					delete calib;

					delete rgb;
					delete depth;
				}
			}

			// Suppress the default copy constructor and assignment operator
			ITMView(const ITMView&);
			ITMView& operator=(const ITMView&);
		};
	}
}

