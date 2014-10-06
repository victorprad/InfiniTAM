// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "OpenNIEngine.h"

#include "../Utils/FileUtils.h"

#include <stdio.h>

#ifndef COMPILE_WITHOUT_OpenNI
#include <OpenNI.h>

using namespace InfiniTAM::Engine;

class OpenNIEngine::PrivateData {
	public:
	PrivateData(void) {}
	openni::Device device;
	openni::VideoStream depthStream, colorStream;

	openni::VideoFrameRef depthFrame;
	openni::VideoFrameRef colorFrame;
	openni::VideoStream **streams;
};

OpenNIEngine::OpenNIEngine(const char *calibFilename, const char *deviceURI, const bool useInternalCalibration)
	: ImageSourceEngine(calibFilename)
{
	data = new PrivateData();
	if (deviceURI==NULL) deviceURI = openni::ANY_DEVICE;

	openni::Status rc = openni::STATUS_OK;

	rc = openni::OpenNI::initialize();
	printf("OpenNI: Initialization ... \n%s\n", openni::OpenNI::getExtendedError());

	rc = data->device.open(deviceURI);
	if (rc != openni::STATUS_OK)
	{
		printf("OpenNI: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return;
	}

	if (useInternalCalibration) data->device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

	rc = data->depthStream.create(data->device, openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK)
	{
		openni::VideoMode depthMode = data->depthStream.getVideoMode();
		depthMode.setResolution(640, 480); depthMode.setFps(30); depthMode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
		rc = data->depthStream.setVideoMode(depthMode);
		if (rc != openni::STATUS_OK)
		{
			printf("OpenNI: Failed to set depth mode\n");
			openni::OpenNI::shutdown();
			return;
		}

		rc = data->depthStream.start();
		if (rc != openni::STATUS_OK)
		{
			printf("OpenNI: Couldn't start depthStream stream:\n%s\n", openni::OpenNI::getExtendedError());
			data->depthStream.destroy();
		}

		depthAvailable = true;
	}
	else
	{
		printf("OpenNI: Couldn't find depthStream stream:\n%s\n", openni::OpenNI::getExtendedError());
		depthAvailable = false;
	}

	rc = data->colorStream.create(data->device, openni::SENSOR_COLOR);
	if (rc == openni::STATUS_OK)
	{
		openni::VideoMode colourMode = data->colorStream.getVideoMode();
		colourMode.setResolution(640, 480); colourMode.setFps(30);
		rc = data->colorStream.setVideoMode(colourMode);
		if (rc != openni::STATUS_OK)
		{
			printf("OpenNI: Failed to set color mode\n");
			openni::OpenNI::shutdown();
			return;
		}

		rc = data->colorStream.start();
		if (rc != openni::STATUS_OK)
		{
			printf("SimpleViewer: Couldn't start colorStream stream:\n%s\n", openni::OpenNI::getExtendedError());
			data->colorStream.destroy();
		}

		colorAvailable = true;
	}
	else
	{
		printf("OpenNI: Couldn't find colorStream stream:\n%s\n", openni::OpenNI::getExtendedError());
		colorAvailable = false;
	}

	if (!depthAvailable)
	{
		printf("OpenNI: No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		return;
	}
	
	data->streams = new openni::VideoStream*[2];
	if (depthAvailable) data->streams[0] = &data->depthStream;
	if (colorAvailable) data->streams[1] = &data->colorStream;
}

OpenNIEngine::~OpenNIEngine()
{
	if (depthAvailable)
	{
		data->depthStream.stop();
		data->depthStream.destroy();
	}
	if (colorAvailable)
	{
		data->colorStream.stop();
		data->colorStream.destroy();
	}
	data->device.close();

	delete[] data->streams;
	delete data;

	openni::OpenNI::shutdown();
}

void OpenNIEngine::getImages(ITMView *out)
{
	int changedIndex, waitStreamCount;
	if (depthAvailable && colorAvailable) waitStreamCount = 2;
	else waitStreamCount = 1;

	openni::Status rc = openni::OpenNI::waitForAnyStream(data->streams, waitStreamCount, &changedIndex);
	if (rc != openni::STATUS_OK) { printf("OpenNI: Wait failed\n"); return /*false*/; }

	if(depthAvailable) data->depthStream.readFrame(&data->depthFrame);
	if(colorAvailable) data->colorStream.readFrame(&data->colorFrame);

	if (depthAvailable && !data->depthFrame.isValid()) return;
	if (colorAvailable && !data->colorFrame.isValid()) return;

	Vector4u *rgb = out->rgb->GetData(false);
	if (colorAvailable)
	{
		const openni::RGB888Pixel* colorImagePix = (const openni::RGB888Pixel*)data->colorFrame.getData();
		for (int i = 0; i < out->rgb->noDims.x * out->rgb->noDims.y; i++)
		{
			Vector4u newPix; openni::RGB888Pixel oldPix = colorImagePix[i];
			newPix.r = oldPix.r; newPix.g = oldPix.g; newPix.b = oldPix.b; newPix.a = 255;
			rgb[i] = newPix;
		}
	}
	else memset(rgb, 0, out->rgb->dataSize * sizeof(Vector4u));

	float *depth = out->depth->GetData(false);
	if (depthAvailable)
	{
		const openni::DepthPixel* depthImagePix = (const openni::DepthPixel*)data->depthFrame.getData();
		for (int i = 0; i < out->depth->noDims.x * out->depth->noDims.y; i++)
		{
			ushort depthPix = depthImagePix[i];
			depth[i] = depthPix == 0 ? -1.0f : (float)depthPix / 1000.0f;
		}
	}
	else memset(depth, 0, out->depth->dataSize * sizeof(short));
	//WriteToTXT((short*)depthImagePix, 307200, "d:/temp/dd.txt");
	//exit(1);

	out->inputImageType = ITMView::InfiniTAM_FLOAT_DEPTH_IMAGE;

	return /*true*/;
}

bool OpenNIEngine::hasMoreImages(void)
{ return true; }
Vector2i OpenNIEngine::getDepthImageSize(void)
{ return Vector2i(640,480); }
Vector2i OpenNIEngine::getRGBImageSize(void)
{ return Vector2i(640,480); }

#else

using namespace InfiniTAM::Engine;

OpenNIEngine::OpenNIEngine(const char *calibFilename, const char *deviceURI, const bool useInternalCalibration)
	: ImageSourceEngine(calibFilename)
{
	printf("compiled without OpenNI support\n");
}
OpenNIEngine::~OpenNIEngine()
{}
void OpenNIEngine::getImages(ITMView *out)
{ return; }
bool OpenNIEngine::hasMoreImages(void)
{ return false; }
Vector2i OpenNIEngine::getDepthImageSize(void)
{ return Vector2i(0,0); }
Vector2i OpenNIEngine::getRGBImageSize(void)
{ return Vector2i(0,0); }

#endif

