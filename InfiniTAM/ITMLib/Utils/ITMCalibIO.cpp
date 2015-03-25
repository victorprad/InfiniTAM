// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMCalibIO.h"

#include <fstream>

using namespace ITMLib;

bool ITMLib::readIntrinsics(std::istream & src, ITMIntrinsics & dest)
{
	float sizeX, sizeY;
	float focalLength[2], centerPoint[2];

	src >> sizeX >> sizeY;
	src >> focalLength[0] >> focalLength[1];
	src >> centerPoint[0] >> centerPoint[1];
	if (src.fail()) return false;

	dest.SetFrom(focalLength[0], focalLength[1], centerPoint[0], centerPoint[1], sizeX, sizeY);
	return true;
}

bool ITMLib::readIntrinsics(const char *fileName, ITMIntrinsics & dest)
{
	std::ifstream f(fileName);
	return ITMLib::readIntrinsics(f, dest);
}

bool ITMLib::readExtrinsics(std::istream & src, ITMExtrinsics & dest)
{
	Matrix4f calib;
	src >> calib.m00 >> calib.m10 >> calib.m20 >> calib.m30;
	src >> calib.m01 >> calib.m11 >> calib.m21 >> calib.m31;
	src >> calib.m02 >> calib.m12 >> calib.m22 >> calib.m32;
	calib.m03 = 0.0f; calib.m13 = 0.0f; calib.m23 = 0.0f; calib.m33 = 1.0f;
	if (src.fail()) return false;

	dest.SetFrom(calib);
	return true;
}

bool ITMLib::readExtrinsics(const char *fileName, ITMExtrinsics & dest)
{
	std::ifstream f(fileName);
	return ITMLib::readExtrinsics(f, dest);
}

bool ITMLib::readDisparityCalib(std::istream & src, ITMDisparityCalib & dest)
{
	float a,b;
	src >> a >> b;
	if (src.fail()) return false;

	dest.SetFrom(a,b);
	return true;
}

bool ITMLib::readDisparityCalib(const char *fileName, ITMDisparityCalib & dest)
{
	std::ifstream f(fileName);
	return ITMLib::readDisparityCalib(f, dest);
}

bool ITMLib::readRGBDCalib(std::istream & src, ITMRGBDCalib & dest)
{
	if (!ITMLib::readIntrinsics(src, dest.intrinsics_rgb)) return false;
	if (!ITMLib::readIntrinsics(src, dest.intrinsics_d)) return false;
	if (!ITMLib::readExtrinsics(src, dest.trafo_rgb_to_depth)) return false;
	if (!ITMLib::readDisparityCalib(src, dest.disparityCalib)) return false;
	return true;
}

bool ITMLib::readRGBDCalib(const char *fileName, ITMRGBDCalib & dest)
{
	std::ifstream f(fileName);
	return ITMLib::readRGBDCalib(f, dest);
}

bool ITMLib::readRGBDCalib(const char *rgbIntrinsicsFile, const char *depthIntrinsicsFile, const char *disparityCalibFile, const char *extrinsicsFile, ITMRGBDCalib & dest)
{
	bool ret = true;
	ret &= ITMLib::readIntrinsics(rgbIntrinsicsFile, dest.intrinsics_rgb);
	ret &= ITMLib::readIntrinsics(depthIntrinsicsFile, dest.intrinsics_d);
	ret &= ITMLib::readExtrinsics(extrinsicsFile, dest.trafo_rgb_to_depth);
	ret &= ITMLib::readDisparityCalib(disparityCalibFile, dest.disparityCalib);
	return ret;
}

