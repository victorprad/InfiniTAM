// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMView.h"
#include "../Misc/ITMIMUMeasurement.h"

namespace ITMLib
{
	/** \brief
	    Represents a single "view", i.e. RGB and depth images along
	    with all intrinsic and relative calibration information
	*/
	class ITMViewIMU : public ITMView
	{
	public:
		ITMIMUMeasurement *imu;

		ITMViewIMU(const ITMRGBDCalib& calibration, Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU)
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
