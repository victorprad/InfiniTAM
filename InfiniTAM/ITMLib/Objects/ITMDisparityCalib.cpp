// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDisparityCalib.h"

using namespace ITMLib::Objects;

ITMDisparityCalib::ITMDisparityCalib(void)
{
	// standard calibration parameters - converts mm to metres by dividing by 1000
	params.x = 0.0f; params.y = 0.0f;
}
