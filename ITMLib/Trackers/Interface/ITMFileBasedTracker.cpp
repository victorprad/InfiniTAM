// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMFileBasedTracker.h"

#include <cstdio>
#include <fstream>

namespace ITMLib {

ITMFileBasedTracker::ITMFileBasedTracker(const std::string &poseMask_, size_t initialFrameNo_) :
		poseMask(poseMask_),
		frameCount(initialFrameNo_)
{}

bool ITMFileBasedTracker::CanKeepTracking() const
{
	std::ifstream poseFile(GetCurrentFilename().c_str());
	return poseFile.is_open();
}

void ITMFileBasedTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
	trackingState->trackerResult = ITMTrackingState::TRACKING_FAILED;

	// Try to open the file
	std::ifstream poseFile(GetCurrentFilename().c_str());

	// Always increment frameCount, this allows skipping missing files that could correspond
	// to frames where tracking failed during capture.
	++frameCount;

	// File not found, signal tracking failure.
	if (!poseFile)
	{
		return;
	}

	Matrix4f invPose;

	// Matrix is column-major
	poseFile >> invPose.m00 >> invPose.m10 >> invPose.m20 >> invPose.m30
	         >> invPose.m01 >> invPose.m11 >> invPose.m21 >> invPose.m31
	         >> invPose.m02 >> invPose.m12 >> invPose.m22 >> invPose.m32
	         >> invPose.m03 >> invPose.m13 >> invPose.m23 >> invPose.m33;

	if (poseFile)
	{
		// No read errors, tracking is assumed good
		trackingState->trackerResult = ITMTrackingState::TRACKING_GOOD;
		trackingState->pose_d->SetInvM(invPose);
	}
}

std::string ITMFileBasedTracker::GetCurrentFilename() const
{
	// Fill the mask
	static const int BUF_SIZE = 2048; // Same as InputSource
	char framePoseFilename[BUF_SIZE];
	sprintf(framePoseFilename, poseMask.c_str(), frameCount);
	return framePoseFilename;
}

}
