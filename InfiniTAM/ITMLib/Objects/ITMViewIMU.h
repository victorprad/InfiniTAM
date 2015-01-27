// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

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

			ITMViewIMU() :ITMView() { }

			~ITMViewIMU(void) { if (isAllocated) delete imu; }

			// Suppress the default copy constructor and assignment operator
			ITMViewIMU(const ITMViewIMU&);
			ITMViewIMU& operator=(const ITMViewIMU&);
		};
	}
}

