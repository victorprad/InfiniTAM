// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMTrackingController.h"

#include "../Objects/ITMRenderState_VH.h"

#include "../ITMLib.h"

using namespace ITMLib::Engine;

void ITMTrackingController::Track(ITMTrackingState *trackingState, const ITMView *view)
{
	tracker->TrackCamera(trackingState, view);
}

void ITMTrackingController::Prepare(ITMTrackingState *trackingState, const ITMView *view)
{
	//render for tracking
	if (settings->trackerType == ITMLibSettings::TRACKER_COLOR)
	{
		ITMPose pose_rgb(view->calib->trafo_rgb_to_depth.calib_inv * trackingState->pose_d->GetM());
		visualisationEngine->CreateExpectedDepths(&pose_rgb, &(view->calib->intrinsics_rgb), renderState_live);
		visualisationEngine->CreatePointCloud(view, trackingState, renderState_live, settings->skipPoints);
	}
	else
	{
		if (trackingState->isKeyFrame)
		{
			visualisationEngine->CreateExpectedDepths(trackingState->pose_d, &(view->calib->intrinsics_d), renderState_live);
			visualisationEngine->CreateICPMaps(view, trackingState, renderState_live);
		}
		else
		{
			visualisationEngine->ForwardRender(view, trackingState, renderState_live);
		}
	}
}