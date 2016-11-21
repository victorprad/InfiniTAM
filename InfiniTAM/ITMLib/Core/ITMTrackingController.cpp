// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMTrackingController.h"

using namespace ITMLib;

void ITMTrackingController::Track(ITMTrackingState *trackingState, const ITMView *view)
{
	if (trackingState->age_pointCloud!=-1) tracker->TrackCamera(trackingState, view);
}
