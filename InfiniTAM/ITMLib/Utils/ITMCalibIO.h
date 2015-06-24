// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Objects/ITMRGBDCalib.h"

#include <iostream>

namespace ITMLib
{
	namespace Objects
	{
		bool readIntrinsics(std::istream & src, ITMIntrinsics & dest);
		bool readIntrinsics(const char *fileName, ITMIntrinsics & dest);
		bool readExtrinsics(std::istream & src, ITMExtrinsics & dest);
		bool readExtrinsics(const char *fileName, ITMExtrinsics & dest);
		bool readDisparityCalib(std::istream & src, ITMDisparityCalib & dest);
		bool readDisparityCalib(const char *fileName, ITMDisparityCalib & dest);
		bool readRGBDCalib(std::istream & src, ITMRGBDCalib & dest);
		bool readRGBDCalib(const char *fileName, ITMRGBDCalib & dest);

		bool readRGBDCalib(const char *rgbIntrinsicsFile, const char *depthIntrinsicsFile, const char *disparityCalibFile, const char *extrinsicsFile, ITMRGBDCalib & dest);
	}
}

