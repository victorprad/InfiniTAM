// authored by https://github.com/ravich2-7183

#include "ROSEngine.h"

#include "../ORUtils/FileUtils.h"

#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <mutex>
#include <functional>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace InputSource;
using namespace sensor_msgs;
using namespace message_filters;

ROSEngine::topicListenerThread()
{
	// subscribe to rgb and depth topics
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub_(nh_, "/dtam/rgb", 1); // TODO remove dtam and generalize to: /namespace/rgb, /namespace/depth
	message_filters::Subscriber<sensor_msgs::Image> depth_sub_(nh_, "/dtam/depth", 1); // uint16 depth image in mm. Native OpenNI format, preferred by InfiniTAM.
	TimeSynchronizer<Image, Image> sync(rgb_sub_, depth_sub_, 10);
	sync.registerCallback(std::bind(&ROSEngine::processMessage, _1, _2));

	ros::spin();	
}

ROSEngine::processMessage(const ImageConstPtr& rgb_image, const ImageConstPtr& depth_image)
{
	std::lock_guard<std::mutex> process_message_lock(images_mutex_);

	// TODO document that rgb_image must have an encoding equivalent to RGBA8
	
	// copy rgb_image into rgb_image_
	uint8 *rgb_image_data = rgb_image.data;
	for (int i = 0; i < rgb_image_->noDims.x * rgb_image_->noDims.y; i++) {
			Vector4u newPix;
			newPix.x = rgb_image_data[i*4+0];
			newPix.y = rgb_image_data[i*4+1];
			newPix.z = rgb_image_data[i*4+2];
			newPix.w = 255;
			
			rgb_image_[i] = newPix;
	}

	// copy depth_image into depth_image_
	const short *depth_image_data = (const short*)depth_image.data;
	memcpy(depth_image_, depth_image_data, depth_image.height*depth_image.step);
}

ROSEngine::ROSEngine(const char *calibFilename, const char *deviceURI, const bool useInternalCalibration,
	Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d) :
	BaseImageSourceEngine(calibFilename),
	nh_(),
	rgb_image_(requested_imageSize_rgb),
	depth_image_(requested_imageSize_d)
{
	// Start up topic listener thread
	std::thread topic_listener_thread(&ROSEngine::topicListenerThread, this);
	topic_listener_thread.join();

	// TODO document that depth images must be in millimeters
	this->calib.disparityCalib.SetStandard(); // assumes depth is in millimeters

	this->imageSize_d	= Vector2i(0,0);
	this->imageSize_rgb = Vector2i(0,0);

	if (useInternalCalibration) { // TODO correct this
		this->calib.trafo_rgb_to_depth = ITMLib::ITMExtrinsics();
		if (depthAvailable) {
			float h_fov = data->depthStream.getHorizontalFieldOfView();
			float v_fov = data->depthStream.getVerticalFieldOfView();
			this->calib.intrinsics_d.SetFrom(
				(float)imageSize_d.x / (2.0f * tan(h_fov/2.0f)),
				(float)imageSize_d.y / (2.0f * tan(v_fov/2.0f)),
				(float)imageSize_d.x / 2.0f,
				(float)imageSize_d.y / 2.0f);
		}
		if (colorAvailable) {
			float h_fov = data->colorStream.getHorizontalFieldOfView();
			float v_fov = data->colorStream.getVerticalFieldOfView();
			this->calib.intrinsics_rgb.SetFrom(
				(float)imageSize_rgb.x / (2.0f * tan(h_fov/2.0f)),
				(float)imageSize_rgb.y / (2.0f * tan(v_fov/2.0f)),
				(float)imageSize_rgb.x / 2.0f,
				(float)imageSize_rgb.y / 2.0f);
		}
	}
}

void ROSEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
	std::lock_guard<std::mutex> get_images_lock(images_mutex_);

	rgbImage->SetFrom(&rgb_image_,       MemoryCopyDirection::CPU_TO_CPU);
	rawDepthImage>SetFrom(&depth_image_, MemoryCopyDirection::CPU_TO_CPU);
}

bool ROSEngine::hasMoreImages(void) const
{
	return ros::ok();
}

Vector2i ROSEngine::getDepthImageSize(void) const
{
	return data ? imageSize_d : Vector2i(0,0);
}

Vector2i ROSEngine::getRGBImageSize(void) const
{
	return data ? imageSize_rgb : Vector2i(0,0);
}

ROSEngine::~ROSEngine()
{
	if (data) {
		delete data;
		data = NULL;
	}
}
