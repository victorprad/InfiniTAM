// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDisparityCalib.h"

using namespace ITMLib::Objects;

ITMDisparityCalib::ITMDisparityCalib(void)
{
	// standard calibration parameters, not very accurate...
	params.x = 1090.f; params.y = 0.075f;
}
