// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMTracker.h"
#include "../../Engines/LowLevel/Interface/ITMLowLevelEngine.h"
#include "../../Objects/Misc/ITMIMUCalibrator.h"
#include "../../Objects/Misc/ITMIMUMeasurement.h"

namespace ITMLib
{
	class ITMIMUTracker : public ITMTracker
	{
	private:
		ITMIMUCalibrator *calibrator;

	public:
		void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);
		bool requiresColourRendering(void) const { return false; }
		bool requiresDepthReliability(void) const { return false; }

		ITMIMUTracker(ITMIMUCalibrator *calibrator);
		virtual ~ITMIMUTracker(void);
	};
}
