// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMMath.h"

namespace ITMLib
{
	class ITMIMUMeasurement
	{
	public:
		Matrix3f R;

		ITMIMUMeasurement()
		{
			this->R.setIdentity();
		}

		ITMIMUMeasurement(const Matrix3f & R)
		{
			this->R = R;
		}

		void SetFrom(const ITMIMUMeasurement *measurement)
		{
			this->R = measurement->R;
		}

		~ITMIMUMeasurement(void) { }

		// Suppress the default copy constructor and assignment operator
		ITMIMUMeasurement(const ITMIMUMeasurement&);
		ITMIMUMeasurement& operator=(const ITMIMUMeasurement&);
	};
}
