// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMIMUTracker.h"
#include "../Objects/ITMViewIMU.h"

using namespace ITMLib::Engine;

ITMIMUTracker::ITMIMUTracker()
{
	this->hasAtLeastTwoFrames = false;

	imuPose_imucoords = new ITMPose();
	imuPose_imucoords->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

	imuPose_cameracoords = new ITMPose();
	imuPose_cameracoords->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

ITMIMUTracker::~ITMIMUTracker(void) 
{ 
	delete imuPose_imucoords;
	delete imuPose_cameracoords;
}

void ITMIMUTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
	imuPose_imucoords->SetR(((ITMViewIMU*)view)->imu->R);

	if (hasAtLeastTwoFrames)
	{
		Matrix3f inv_oldR_visual; oldR_visual.inv(inv_oldR_visual);
		Vector3f t_imu, r_imu;

		imuPose_cameracoords->SetR(imuPose_imucoords->GetR() * inv_oldR_visual);
		imuPose_cameracoords->GetParams(t_imu, r_imu);
		imuPose_cameracoords->SetFrom(t_imu.x, t_imu.y, t_imu.z, r_imu.y, r_imu.z, -r_imu.x);

		trackingState->pose_d->SetR(imuPose_cameracoords->GetR() * trackingState->pose_d->GetR());
	}

	oldR_visual = imuPose_imucoords->GetR();
	hasAtLeastTwoFrames = true;
}