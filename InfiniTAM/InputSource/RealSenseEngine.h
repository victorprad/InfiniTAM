// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#ifdef COMPILE_WITH_RealSense
#include "librealsense/rs.hpp"
#endif

#include "ImageSourceEngine.h"

#if (!defined USING_CMAKE) && (defined _MSC_VER)
#ifdef _DEBUG
#pragma comment(lib, "libpxcmd_d")
#else
#pragma comment(lib, "libpxcmd")
#endif 
#endif

namespace InputSource {

class RealSenseEngine : public BaseImageSourceEngine
{
private:
	class PrivateData;
	PrivateData *data; bool dataAvailable;

	Vector2i imageSize_rgb, imageSize_d;

#ifdef COMPILE_WITH_RealSense
	rs::stream colourStream;
#endif

public:
	RealSenseEngine(const char *calibFilename, bool alignColourWithDepth = true,
	                Vector2i imageSize_rgb = Vector2i(640, 480), Vector2i imageSize_d = Vector2i(640, 480));
	~RealSenseEngine();

	bool hasMoreImages(void) const;
	void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);
	Vector2i getDepthImageSize(void) const;
	Vector2i getRGBImageSize(void) const;
};

}
