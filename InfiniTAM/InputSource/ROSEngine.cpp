// authored by https://github.com/ravich2-7183

#include "ROSEngine.h"

using namespace std;
using namespace ORUtils;
using namespace InputSource;
using namespace sensor_msgs;
using namespace message_filters;

void ROSEngine::processMessage(const ImageConstPtr& rgb_image_msg, const ImageConstPtr& depth_image_msg)
{
	std::lock_guard<std::mutex> process_message_lock(images_mutex_);

	// copy rgb_image_msg into rgb_image_
	Vector4u *rgb = rgb_image_.GetData(MEMORYDEVICE_CPU);
	for(int i = 0; i < rgb_image_.noDims.x * rgb_image_.noDims.y; i++) {
		Vector4u newPix;
		newPix.x = (rgb_image_msg->data)[i*3+2];
		newPix.y = (rgb_image_msg->data)[i*3+1];
		newPix.z = (rgb_image_msg->data)[i*3+0];
		newPix.w = 255;

		rgb[i] = newPix;
	}

	// copy depth_image_msg into depth_image_
	short *depth = depth_image_.GetData(MEMORYDEVICE_CPU);
	const short *depth_msg_data  = reinterpret_cast<const short*>(depth_image_msg->data.data());
	for(int i = 0; i < depth_image_.noDims.x * depth_image_.noDims.y; i++) {
		depth[i] = depth_msg_data[i];
	}
}

void ROSEngine::topicListenerThread()
{
	// subscribe to rgb and depth topics
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub_(nh_, "/camera/rgb/image_color", 1);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub_(nh_, "/camera/depth/image_raw", 1); 
	typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ITAMSyncPolicy;
	Synchronizer<ITAMSyncPolicy> sync(ITAMSyncPolicy(10), rgb_sub_, depth_sub_);
	sync.registerCallback(boost::bind(&ROSEngine::processMessage, this, _1, _2));

	ros::spin();
}

ROSEngine::ROSEngine(const char *calibFilename,
					 Vector2i requested_imageSize_rgb,
					 Vector2i requested_imageSize_d) :
			BaseImageSourceEngine(calibFilename),
			nh_(),
			rgb_image_(requested_imageSize_rgb, MEMORYDEVICE_CPU),
			depth_image_(requested_imageSize_d, MEMORYDEVICE_CPU),
			topic_listener_thread(&ROSEngine::topicListenerThread, this) // Starts up topicListenerThread
{
	this->calib.disparityCalib.SetStandard(); // assumes depth is in millimeters
}

void ROSEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
	std::lock_guard<std::mutex> get_images_lock(images_mutex_);

	rgbImage->SetFrom(&rgb_image_,        MemoryBlock<Vector4u>::CPU_TO_CPU); 
	rawDepthImage->SetFrom(&depth_image_, MemoryBlock<short>::CPU_TO_CPU);
}

bool ROSEngine::hasMoreImages(void) const
{
	return ros::ok();
}

Vector2i ROSEngine::getDepthImageSize(void) const
{
	return Vector2i(depth_image_.noDims.x , depth_image_.noDims.y);
}

Vector2i ROSEngine::getRGBImageSize(void) const
{
	return Vector2i(rgb_image_.noDims.x , rgb_image_.noDims.y);
}

ROSEngine::~ROSEngine()
{}
