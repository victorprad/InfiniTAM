// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMIntrinsics.h"

#include <fstream>

using namespace ITMLib::Objects;

ITMIntrinsics::ITMIntrinsics(void) 
{
	// standard calibration parameters for Kinect RGB camera. Not at all
	// accurate, though...
	SetFrom(580, 580, 320, 240, 640, 480);
}

void ITMIntrinsics::SetFrom(float fx, float fy, float cx, float cy, float sizeX, float sizeY)
{
	projectionParamsSimple.fx = fx; projectionParamsSimple.fy = fy;
	projectionParamsSimple.px = cx; projectionParamsSimple.py = cy;
	projectionParamsSimple.all.x = fx; projectionParamsSimple.all.y = fy;
	projectionParamsSimple.all.z = cx; projectionParamsSimple.all.w = cy;
}

