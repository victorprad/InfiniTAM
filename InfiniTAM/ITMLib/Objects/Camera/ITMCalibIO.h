// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <iosfwd>

#include "ITMRGBDCalib.h"

namespace ITMLib
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
	void writeIntrinsics(std::ostream & dest, const ITMIntrinsics & src);
	void writeExtrinsics(std::ostream & dest, const ITMExtrinsics & src);
	void writeDisparityCalib(std::ostream & dest, const ITMDisparityCalib & src);
	void writeRGBDCalib(std::ostream & dest, const ITMRGBDCalib & src);
	void writeRGBDCalib(const char *fileName, const ITMRGBDCalib & src);
}
