// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMLibSettings.h"

#include <stdio.h>

using namespace ITMLib::Objects;

ITMLibSettings::ITMLibSettings(void)
	: sceneParams(0.02f, 100, 0.005f, 0.2f, 3.0f)
{
	noHierarchyLevels = 5;
	noRotationOnlyLevels = 3;

	/// depth threashold for the ICP tracker
	depthTrackerICPThreshold = 0.1f * 0.1f;

	/// skips every other point when using the colour tracker
	skipPoints = true;

#ifndef COMPILE_WITHOUT_CUDA
	useGPU = true;
#else
	useGPU = false;
#endif

	/// enables or disables swapping. HERE BE DRAGONS: It should work, but requires more testing
	useSwapping = false;

	//trackerType = TRACKER_COLOR;
	trackerType = TRACKER_ICP;

	/** Use ITMRenTracker to reduce the wiggling when the depth sensor has missing data
	Using this option, the tracking will frist start ICP from higher hierarchy for
	initialial pose estimation, then run Ren's tracker at lowest level of hierachy.
	The processing time on each frame is increased by around 50% (~5ms on a GTX Titan Black)**/

	//trackerType = TRACKER_REN;

	if (trackerType == TRACKER_REN) noICPRunTillLevel = 1;
	else noICPRunTillLevel = 0;

	if ((trackerType == TRACKER_COLOR) && (!ITMVoxel::hasColorInformation)) {
		printf("Error: Color tracker requires a voxel type with color information!\n");
	}
}
