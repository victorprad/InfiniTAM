// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <mutex>

#include "ImageSourceEngine.h"

#if (!defined USING_CMAKE) && (defined _MSC_VER)
#pragma comment(lib, "OpenNI2")
#endif

namespace InputSource {

class ROSEngine : public BaseImageSourceEngine
{
private:
	ros::NodeHandle nh_;
	ros::Subscriber rgb_sub_, depth_sub_;
	Vector2i imageSize_rgb_, imageSize_d_;
	ITMUChar4Image rgb_image_;
	ITMShortImage depth_image_;
	std::mutex images_mutex_;

public:
	ROSEngine(const char *calibFilename, const char *deviceURI = NULL, const bool useInternalCalibration = false,
		Vector2i imageSize_rgb = Vector2i(640, 480), Vector2i imageSize_d = Vector2i(640, 480));
	~ROSEngine();

	bool hasMoreImages(void) const;
	void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);
	Vector2i getDepthImageSize(void) const;
	Vector2i getRGBImageSize(void) const;
};

}
