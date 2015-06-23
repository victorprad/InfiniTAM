// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Objects/ITMView.h"
#include "../Objects/ITMIMUMeasurement.h"

namespace ITMLib
{
	namespace Objects
	{
		/** \brief
		    Represents a single "view", i.e. RGB and depth images along
		    with all intrinsic and relative calibration information
		*/
		class ITMViewIMU : public ITMView
		{
		public:
			ITMIMUMeasurement *imu;

			ITMViewIMU(const ITMRGBDCalib *calibration, Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU)
			 : ITMView(calibration, imgSize_rgb, imgSize_d, useGPU)
			{
				imu = new ITMIMUMeasurement();
			}

			~ITMViewIMU(void) { delete imu; }

			// Suppress the default copy constructor and assignment operator
			ITMViewIMU(const ITMViewIMU&);
			ITMViewIMU& operator=(const ITMViewIMU&);
		};
	}
}

