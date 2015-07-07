// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMTrackingController.h"

#include "../Objects/ITMRenderState_VH.h"

#include "../ITMLib.h"

using namespace ITMLib;

void ITMTrackingController::Track(ITMTrackingState *trackingState, const ITMView *view)
{
	if (trackingState->age_pointCloud!=-1) tracker->TrackCamera(trackingState, view);
}

void ITMTrackingController::Prepare(ITMTrackingState *trackingState, const ITMSceneBase *scene, const ITMView *view, ITMRenderState *renderState)
{
	//render for tracking
	bool requiresColourRendering = tracker->requiresColourRendering();
	bool requiresFullRendering = trackingState->TrackerFarFromPointCloud() || !settings->useApproximateRaycast;

	if (requiresColourRendering)
	{
		ITMPose pose_rgb(view->calib->trafo_rgb_to_depth.calib_inv * trackingState->pose_d->GetM());
		visualisationEngine->CreateExpectedDepths(scene, &pose_rgb, &(view->calib->intrinsics_rgb), renderState);
		visualisationEngine->CreatePointCloud(scene, view, trackingState, renderState, settings->skipPoints);
		trackingState->age_pointCloud = 0;
	}
	else
	{
		visualisationEngine->CreateExpectedDepths(scene, trackingState->pose_d, &(view->calib->intrinsics_d), renderState);

		if (requiresFullRendering)
		{
			visualisationEngine->CreateICPMaps(scene, view, trackingState, renderState);
			trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
			if (trackingState->age_pointCloud==-1) trackingState->age_pointCloud=-2;
			else trackingState->age_pointCloud = 0;
		}
		else
		{
			visualisationEngine->ForwardRender(scene, view, trackingState, renderState);
			trackingState->age_pointCloud++;
		}
	}
}
