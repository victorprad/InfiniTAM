// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMIMUTracker.h"
#include "../Objects/ITMViewIMU.h"

using namespace ITMLib::Engine;

ITMIMUTracker::ITMIMUTracker(ITMIMUCalibrator *calibrator)
{
	this->calibrator = calibrator;
}

ITMIMUTracker::~ITMIMUTracker(void)
{
}

void ITMIMUTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
	calibrator->RegisterMeasurement(((ITMViewIMU*)view)->imu->R);

	trackingState->pose_d->SetR(calibrator->GetDifferentialRotationChange() * trackingState->pose_d->GetR());
}