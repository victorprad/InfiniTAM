// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "RealSenseEngine.h"

#include "../Utils/FileUtils.h"

#include <cstdio>
#include <stdexcept>

#ifdef COMPILE_WITH_RealSenseWindows

#include <pxcsensemanager.h>

using namespace InfiniTAM::Engine;

class RealSenseEngine::PrivateData {
public:
	PrivateData(void) : pxcSenseManager(NULL){}
	PXCSenseManager *pxcSenseManager = NULL;
};

RealSenseEngine::RealSenseEngine(const char *calibFilename, Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d)
	: ImageSourceEngine(calibFilename)
{
	// images from openni always come in millimeters...
	this->calib.disparityCalib.type = ITMDisparityCalib::TRAFO_AFFINE;
	this->calib.disparityCalib.params = Vector2f(1.0f/1000.0f, 0.0f);
	
	this->calib.trafo_rgb_to_depth = ITMExtrinsics();
	this->calib.intrinsics_d = this->calib.intrinsics_rgb;

	this->imageSize_d = requested_imageSize_d;
	this->imageSize_rgb = requested_imageSize_rgb;

	data = new RealSenseEngine::PrivateData();

	data->pxcSenseManager = PXCSenseManager::CreateInstance();

	//data->pxcSenseManager->EnableStream(PXCCapture::STREAM_TYPE_COLOR, requested_imageSize_rgb.x, requested_imageSize_rgb.y, 30);
	data->pxcSenseManager->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, requested_imageSize_d.x, requested_imageSize_d.y, 90);

	if (data->pxcSenseManager->Init()<PXC_STATUS_NO_ERROR) 
	{
		wprintf_s(L"Failed to locate any video stream(s)\n");
		dataAvailable = false;

		data->pxcSenseManager->Release();
		delete data;
		data = NULL;
	}
}

RealSenseEngine::~RealSenseEngine()
{
	if (data != NULL)
	{
		data->pxcSenseManager->Release();
	}
	delete data;
}


void RealSenseEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
	dataAvailable = false;

	if (data->pxcSenseManager->AcquireFrame(false) < PXC_STATUS_NO_ERROR) return;

	PXCCapture::Sample *sample = data->pxcSenseManager->QuerySample();

	if (sample)
	{
		//if (sample->color)
		//{
		//	PXCImage::ImageData imageData;
		//	sample->color->AcquireAccess(PXCImage::ACCESS_READ, &imageData);
		//	sample->color->ReleaseAccess(&imageData);
		//}

		if (sample->depth)
		{
			PXCImage::ImageData imageData;
			sample->depth->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PixelFormat::PIXEL_FORMAT_DEPTH, &imageData);
			

			short *rawDepth = rawDepthImage->GetData(MEMORYDEVICE_CPU);

			rawDepthImage->Clear();

			short *rsData = (short*)imageData.planes[0];
			Vector2i noDims = rawDepthImage->noDims;
			int stride = imageData.pitches[0] / sizeof(short);

			for (int y = 0; y < noDims.y; y++) for (int x = 0; x < noDims.x; x++)
				rawDepth[x + y * noDims.x] = rsData[x + y * stride];

			sample->depth->ReleaseAccess(&imageData);
		}
		dataAvailable = true;
	}

	data->pxcSenseManager->ReleaseFrame();
}

bool RealSenseEngine::hasMoreImages(void) { return (data!=NULL); }
Vector2i RealSenseEngine::getDepthImageSize(void) { return (data!=NULL)?imageSize_d:Vector2i(0,0); }
Vector2i RealSenseEngine::getRGBImageSize(void) { return (data!=NULL)?imageSize_rgb:Vector2i(0,0); }

#else

using namespace InfiniTAM::Engine;

RealSenseEngine::RealSenseEngine(const char *calibFilename, Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d)
	: ImageSourceEngine(calibFilename)
{
	printf("compiled without RealSense Windows support\n");
}
RealSenseEngine::~RealSenseEngine()
{}
void RealSenseEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{ return; }
bool RealSenseEngine::hasMoreImages(void)
{ return false; }
Vector2i RealSenseEngine::getDepthImageSize(void)
{ return Vector2i(0,0); }
Vector2i RealSenseEngine::getRGBImageSize(void)
{ return Vector2i(0,0); }

#endif

