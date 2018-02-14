// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMCalibIO.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

using namespace ITMLib;

bool ITMLib::readIntrinsics(std::istream & src, ITMIntrinsics & dest)
{
	int width, height;
	float focalLength[2], centerPoint[2];

	src >> width >> height;
	src >> focalLength[0] >> focalLength[1];
	src >> centerPoint[0] >> centerPoint[1];
	if (src.fail()) return false;

	dest.SetFrom(width, height, focalLength[0], focalLength[1], centerPoint[0], centerPoint[1]);
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
	std::string word;
	src >> word;
	if (src.fail()) return false;

	ITMDisparityCalib::TrafoType type = ITMDisparityCalib::TRAFO_KINECT;
	float a,b;

	if (word.compare("kinect") == 0) {
		type = ITMDisparityCalib::TRAFO_KINECT;
		src >> a;
	} else if (word.compare("affine") == 0) {
		type = ITMDisparityCalib::TRAFO_AFFINE;
		src >> a;
	} else {
		std::stringstream wordstream(word);
		wordstream >> a;
		if (wordstream.fail()) return false;
	}

	src >> b;
	if (src.fail()) return false;

	if ((a == 0.0f) && (b == 0.0f)) {
		type = ITMDisparityCalib::TRAFO_AFFINE;
		a = 1.0f/1000.0f; b = 0.0f;
	}

	dest.SetFrom(a, b, type);
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

void ITMLib::writeIntrinsics(std::ostream & dest, const ITMIntrinsics & src)
{
	dest << src.imgSize.width << ' ' << src.imgSize.height << '\n';
	dest << src.projectionParamsSimple.fx << ' ' << src.projectionParamsSimple.fy << '\n';
	dest << src.projectionParamsSimple.px << ' ' << src.projectionParamsSimple.py << '\n';
}

void ITMLib::writeExtrinsics(std::ostream & dest, const ITMExtrinsics & src)
{
	const Matrix4f& calib = src.calib;
	dest << calib.m00 << ' ' << calib.m10 << ' ' << calib.m20 << ' ' << calib.m30 << '\n';
	dest << calib.m01 << ' ' << calib.m11 << ' ' << calib.m21 << ' ' << calib.m31 << '\n';
	dest << calib.m02 << ' ' << calib.m12 << ' ' << calib.m22 << ' ' << calib.m32 << '\n';
}

void ITMLib::writeDisparityCalib(std::ostream & dest, const ITMDisparityCalib & src)
{
	switch(src.GetType())
	{
		case ITMDisparityCalib::TRAFO_AFFINE:
			dest << "affine " << src.GetParams().x << ' ' << src.GetParams().y << '\n';
			break;
		case ITMDisparityCalib::TRAFO_KINECT:
			dest << src.GetParams().x << ' ' << src.GetParams().y << '\n';
			break;
		default:
			throw std::runtime_error("Error: Unknown disparity calibration type");
	}
}

void ITMLib::writeRGBDCalib(std::ostream & dest, const ITMRGBDCalib & src)
{
	writeIntrinsics(dest, src.intrinsics_rgb);
	dest << '\n';
	writeIntrinsics(dest, src.intrinsics_d);
	dest << '\n';
	writeExtrinsics(dest, src.trafo_rgb_to_depth);
	dest << '\n';
	writeDisparityCalib(dest, src.disparityCalib);
}

void ITMLib::writeRGBDCalib(const char *fileName, const ITMRGBDCalib & src)
{
	std::ofstream f(fileName);
	writeRGBDCalib(f, src);
}
