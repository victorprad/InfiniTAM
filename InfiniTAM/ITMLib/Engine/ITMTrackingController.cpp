// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMTrackingController.h"

#include "../Objects/ITMRenderState_VH.h"

#include "../ITMLib.h"

using namespace ITMLib::Engine;

void ITMTrackingController::Track(ITMTrackingState *trackingState, const ITMView *view)
{
	tracker->TrackCamera(trackingState, view);
	noTrackedFrames++;
}

bool ITMTrackingController::IsFarFromPrevious(ITMTrackingState *trackingState)
{
	if (!hasPrevPose || (noTrackedFrames < 2)) return true;

	Vector3f cameraCenter_pc = -1.0f * (this->prevPose->GetR().t() * this->prevPose->GetT());
	Vector3f cameraCenter_live = -1.0f * (trackingState->pose_d->GetR().t() * trackingState->pose_d->GetT());

	Vector3f diff3 = cameraCenter_pc - cameraCenter_live;

	float diff = diff3.x * diff3.x + diff3.y * diff3.y + diff3.z * diff3.z;

	if (diff > 0.0002f && noFramesForLastIntegration > 5) return true;

	return false;
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
		visualisationEngine->CreateExpectedDepths(trackingState->pose_d, &(view->calib->intrinsics_d), renderState_live);

		if (this->IsFarFromPrevious(trackingState) || !useApproximateRaycast)
		{
			visualisationEngine->CreateICPMaps(view, trackingState, renderState_live);
			this->prevPose->SetFrom(trackingState->pose_d);
			noFramesForLastIntegration = 0;
		}
		else
		{
			visualisationEngine->ForwardRender(view, trackingState, renderState_live);
			noFramesForLastIntegration++;
		}
	}

	this->hasPrevPose = true;
}