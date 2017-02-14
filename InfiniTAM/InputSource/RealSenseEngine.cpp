// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "RealSenseEngine.h"

#include "../ORUtils/FileUtils.h"

#include <cstdio>
#include <stdexcept>

#ifdef COMPILE_WITH_RealSense

#include "librealsense/rs.hpp"

using namespace InputSource;
using namespace ITMLib;

class RealSenseEngine::PrivateData
{
	public:
	PrivateData(void) : dev(NULL){}
	rs::device *dev = NULL;
	rs::context ctx;
};

RealSenseEngine::RealSenseEngine(const char *calibFilename, Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d)
	: BaseImageSourceEngine(calibFilename)
{
	this->calib.disparityCalib.SetStandard();
	this->calib.trafo_rgb_to_depth = ITMExtrinsics();
	this->calib.intrinsics_d = this->calib.intrinsics_rgb;

	this->imageSize_d = requested_imageSize_d;
	this->imageSize_rgb = requested_imageSize_rgb;

	data = new RealSenseEngine::PrivateData();
	printf("There are %d connected RealSense devices.\n", data->ctx.get_device_count());
	if (data->ctx.get_device_count() == 0) {
		dataAvailable = false;
		delete data;
		data = NULL;
		return;
	}

	data->dev = data->ctx.get_device(0);

	data->dev->enable_stream(rs::stream::depth, imageSize_d.x, imageSize_d.y, rs::format::z16, 60);
	data->dev->enable_stream(rs::stream::color, imageSize_rgb.x, imageSize_rgb.y, rs::format::rgb8, 60);

	rs::intrinsics intrinsics_depth = data->dev->get_stream_intrinsics(rs::stream::depth);
	rs::intrinsics intrinsics_rgb = data->dev->get_stream_intrinsics(rs::stream::color);

	this->calib.intrinsics_d.SetFrom(intrinsics_depth.fx, intrinsics_depth.fy,
	                                 intrinsics_depth.ppx, intrinsics_depth.ppy);
	this->calib.intrinsics_rgb.SetFrom(intrinsics_rgb.fx, intrinsics_rgb.fy,
	                                 intrinsics_rgb.ppx, intrinsics_rgb.ppy);
	this->calib.disparityCalib.SetFrom(data->dev->get_depth_scale(), 0.0f,
		ITMLib::ITMDisparityCalib::TRAFO_AFFINE);

	data->dev->start();
}

RealSenseEngine::~RealSenseEngine()
{
	if (data != NULL)
	{
		data->dev->stop();
		delete data;
	}
}


void RealSenseEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
	dataAvailable = false;

	// get frames
	data->dev->wait_for_frames();
	const uint16_t * depth_frame = reinterpret_cast<const uint16_t *>(data->dev->get_frame_data(rs::stream::depth));
	const uint8_t * color_frame = reinterpret_cast<const uint8_t*>(data->dev->get_frame_data(rs::stream::color));

	// setup infinitam frames
	short *rawDepth = rawDepthImage->GetData(MEMORYDEVICE_CPU);
	Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);

	Vector2i noDims = rawDepthImage->noDims;
	rawDepthImage->Clear();
	rgbImage->Clear();

	for (int y = 0; y < noDims.y; y++) for (int x = 0; x < noDims.x; x++) rawDepth[x + y * noDims.x] = *depth_frame++;

	for (int i = 0; i < rgbImage->noDims.x * 3 * rgbImage->noDims.y ; i+=3) {
		Vector4u newPix;
		newPix.x = color_frame[i]; newPix.y = color_frame[i+1]; newPix.z = color_frame[i+2];
		newPix.w = 255;
		rgb[i/3] = newPix;
	}

	dataAvailable = true;
}

bool RealSenseEngine::hasMoreImages(void) const { return (data!=NULL); }
Vector2i RealSenseEngine::getDepthImageSize(void) const { return (data!=NULL)?imageSize_d:Vector2i(0,0); }
Vector2i RealSenseEngine::getRGBImageSize(void) const { return (data!=NULL)?imageSize_rgb:Vector2i(0,0); }

#else

using namespace InputSource;

RealSenseEngine::RealSenseEngine(const char *calibFilename, Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d)
	: BaseImageSourceEngine(calibFilename)
{
	printf("compiled without RealSense Windows support\n");
}
RealSenseEngine::~RealSenseEngine()
{}
void RealSenseEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{ return; }
bool RealSenseEngine::hasMoreImages(void) const
{ return false; }
Vector2i RealSenseEngine::getDepthImageSize(void) const
{ return Vector2i(0,0); }
Vector2i RealSenseEngine::getRGBImageSize(void) const
{ return Vector2i(0,0); }

#endif

