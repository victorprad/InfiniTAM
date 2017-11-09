// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../ORUtils/FileUtils.h"
#include "ImageSourceEngine.h"

#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
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
	std::thread topic_listener_thread;
	std::mutex images_mutex_;

public:
	ROSEngine(const char *calibFilename, 
			  Vector2i imageSize_rgb = Vector2i(640, 480),
			  Vector2i imageSize_d = Vector2i(640, 480));
	~ROSEngine();

	/**
	   @param rgb_image_msg must have an encoding "bgr8"
	   @param depth_image_msg must have an encoding "mono16" or "16UC1" which is the depth in mm. 
	*/
	void processMessage(const sensor_msgs::ImageConstPtr& rgb_image_msg, const sensor_msgs::ImageConstPtr& depth_image_msg);

	void topicListenerThread();
	
	bool hasMoreImages(void) const;
	void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);
	Vector2i getDepthImageSize(void) const;
	Vector2i getRGBImageSize(void) const;
};

}
