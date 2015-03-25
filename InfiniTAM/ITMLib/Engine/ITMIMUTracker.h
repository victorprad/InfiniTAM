// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMIMUMeasurement.h"

#include "../Engine/ITMTracker.h"
#include "../Engine/ITMLowLevelEngine.h"
#include "../Engine/ITMIMUCalibrator.h"

namespace ITMLib
{
	class ITMIMUTracker : public ITMTracker
	{
	private:
		ITMIMUCalibrator *calibrator;

	public:
		void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

		ITMIMUTracker(ITMIMUCalibrator *calibrator);
		virtual ~ITMIMUTracker(void);
	};
}
