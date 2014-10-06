// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMLibSettings.h"

using namespace ITMLib::Objects;

ITMLibSettings::ITMLibSettings(void)
 : sceneParams(0.02f, 100, 0.005f, 0.2f, 3.0f)
{
	noHierarchyLevels = 5;
	noRotationOnlyLevels = 3;

	depthTrackerICPThreshold = 0.1f * 0.1f;

	skipPoints = true;
#ifndef COMPILE_WITHOUT_CUDA
	useGPU = true;
#else
	useGPU = false;
#endif

	useSwapping = false; // HERE BE DRAGONS: It should work, but requires more testing

	//trackerType = TRACKER_COLOR;
	trackerType = TRACKER_ICP;
}
