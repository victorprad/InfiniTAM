// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMLibSettings.h"
using namespace ITMLib;

#include <climits>
#include <cmath>

ITMLibSettings::ITMLibSettings(void)
:	sceneParams(0.02f, 100, 0.004f, 0.2f, 3.0f, false),
	surfelSceneParams(0.5f, 0.6f, static_cast<float>(20 * M_PI / 180), 0.01f, 0.004f, 3.5f, 25.0f, 4, 1.0f, 5.0f, 20, 10000000, true, true)
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
	useApproximateRaycast = true;

	/// enable or disable bilateral depth filtering;
	useBilateralFilter = false;

	/// enable or disable relocalisation
	useRelocalisation = true;

	/// enable or disable tracking failure detection
	useTrackingFailureDetection = true;

	//trackerConfig = "type=icp,levels=rrrbb,minstep=1e-3,outlierC=0.01,outlierF=0.002,numiterC=10,numiterF=2,failureDec=3.0";
	//trackerConfig = "type=icp,levels=rrrbb,minstep=1e-2,outlierC=0.1,outlierF=0.005,numiterC=10,numiterF=2,failureDec=3.0";
	trackerConfig = "type=extended,levels=rrbb,minstep=1e-4,outlierSpaceC=0.1,outlierSpaceF=0.004,numiterC=20,numiterF=20,tukeyCutOff=8,framesToSkip=20,framesToWeight=50,failureDec=20.0";
	//trackerConfig = "type=rgb,levels=rrrbb";
	//trackerConfig = "type=ren,levels=bb";
	//trackerConfig = "type=wicp,levels=rrrbb,minstep=1e-3,outlier=0.005";
	//trackerConfig = "type=imuicp,levels=tb,minstep=1e-3,outlierC=0.01,outlierF=0.005,numiterC=4,numiterF=2";
}