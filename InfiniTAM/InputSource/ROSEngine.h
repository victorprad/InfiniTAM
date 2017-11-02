// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../ORUtils/FileUtils.h"
#include "ImageSourceEngine.h"

#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#if (!defined USING_CMAKE) && (defined _MSC_VER)
#pragma comment(lib, "OpenNI2")
#endif

namespace InputSource {

class ROSEngine : public BaseImageSourceEngine
{
private:
	ros::NodeHandle nh_;
	ros::Subscriber rgb_sub_, depth_sub_;
	ITMUChar4Image rgb_image_;
	ITMShortImage depth_image_;
	std::mutex images_mutex_;

public:
	ROSEngine(const char *calibFilename, 
			  Vector2i imageSize_rgb = Vector2i(640, 480),
			  Vector2i imageSize_d = Vector2i(640, 480));
	~ROSEngine();

	void processMessage(const sensor_msgs::ImageConstPtr& rgb_image, const sensor_msgs::ImageConstPtr& depth_image);
	void topicListenerThread();
	
	bool hasMoreImages(void) const;
	void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);
	Vector2i getDepthImageSize(void) const;
	Vector2i getRGBImageSize(void) const;
};

}
