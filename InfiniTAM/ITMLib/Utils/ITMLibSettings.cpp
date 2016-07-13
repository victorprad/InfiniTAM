// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMLibSettings.h"
using namespace ITMLib;

#include <climits>
#include <cmath>

ITMLibSettings::ITMLibSettings(void)
:	sceneParams(0.02f, 100, 0.005f, 0.2f, 3.0f, false),
	surfelSceneParams(0.5f, 0.6f, static_cast<float>(20 * M_PI / 180), 0.01f, 0.004f, 3.5f, 25.0f, 4, 1.0f, 5.0f, 20, 10000000, true, true)
{
	// skips every other point when using the colour renderer for creating a point cloud
	skipPoints = true;

	// create all the things required for marching cubes and mesh extraction
	// - uses additional memory (lots!)
	createMeshingEngine = true;
    
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

	/// enables or disables swapping. HERE BE DRAGONS: It should work, but requires more testing
	useSwapping = false;

	/// enables or disables approximate raycast
	useApproximateRaycast = false;

	/// enable or disable bilateral depth filtering
	useBilateralFilter = false;

	/// what to do on tracker failure: ignore, relocalise or stop integration
	behaviourOnFailure = FAILUREMODE_IGNORE;
    
	trackerConfig = "type=icp,levels=rrrbb,minstep=1e-3,outlierC=0.01,outlierF=0.002,numiterC=10,numiterF=2,failureDec=3.0";
	//trackerConfig = "type=extended,levels=rrbb,minstep=1e-4,outlierSpaceC=0.1,outlierSpaceF=0.004,numiterC=20,numiterF=50,tukeyCutOff=8,framesToSkip=20,framesToWeight=50,failureDec=20.0";
	//trackerConfig = "type=rgb,levels=rrrbb";
	//trackerConfig = "type=ren,levels=bb";
	//trackerConfig = "type=imuicp,levels=tb,minstep=1e-3,outlierC=0.01,outlierF=0.005,numiterC=4,numiterF=2";
	//trackerConfig = "type=extendedimu,levels=ttb,minstep=5e-4,outlierSpaceC=0.1,outlierSpaceF=0.004,numiterC=20,numiterF=5,tukeyCutOff=8,framesToSkip=20,framesToWeight=50,failureDec=20.0";
}
