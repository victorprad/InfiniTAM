// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMTrackingController.h"

#include "../Objects/ITMRenderState_VH.h"

#include "../ITMLib.h"

using namespace ITMLib;

void ITMTrackingController::Track(ITMTrackingState *trackingState, const ITMView *view)
{
	if (trackingState->age_pointCloud!=-1) tracker->TrackCamera(trackingState, view);
}
