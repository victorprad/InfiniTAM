// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMLibSettings.h"
using namespace ITMLib;

ITMLibSettings::ITMLibSettings(void)
	: sceneParams(0.02f, 100, 0.005f, 0.2f, 4.0f, false)
{
	// skips every other point when using the colour renderer for creating a point cloud
	skipPoints = true;

#ifndef COMPILE_WITHOUT_CUDA
	deviceType = DEVICE_CUDA;
#else
#ifdef COMPILE_WITH_METAL
	deviceType = DEVICE_METAL;
#else
	deviceType = DEVICE_CPU;
#endif
#endif

	//deviceType = DEVICE_CPU;

	goodTrackingThreshold = 0.8f;
	poorTrackingThreshold = 0.4f;

	/// enables or disables swapping. HERE BE DRAGONS: It should work, but requires more testing
	useSwapping = false;

	/// enables or disables approximate raycast
	useApproximateRaycast = false;

	/// enable or disable bilateral depth filtering;
	useBilateralFilter = false;

	/// enable or disable relocalisation
	useRelocalisation = true;

	/// enable or disable tracking failure detection
	useTrackingFailureDetection = true;

	trackerConfig = "type=icp,levels=rrrbb,minstep=1e-3,outlierC=0.01,outlierF=0.002,numiterC=10,numiterF=2";
	//trackerConfig = "type=rgb,levels=rrrbb";
	//trackerConfig = "type=ren,levels=bb";
	//trackerConfig = "type=wicp,levels=rrrbb,minstep=1e-3,outlier=0.01";
	//trackerConfig = "type=imuicp,levels=tb,minstep=1e-3,outlierC=0.01,outlierF=0.005,numiterC=4,numiterF=2";
}

