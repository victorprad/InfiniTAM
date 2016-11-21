// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "Kinect2Engine.h"

#include "../ORUtils/FileUtils.h"

#include <stdio.h>

#ifndef COMPILE_WITHOUT_Kinect2
#include <Kinect.h>

#pragma comment(lib, "kinect20.lib")

using namespace InputSource;

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

class Kinect2Engine::PrivateData {
	public:
	PrivateData(void) {}

	IKinectSensor* kinectSensor;
	IDepthFrameReader* depthFrameReader;
};

Kinect2Engine::Kinect2Engine(const char *calibFilename) : BaseImageSourceEngine(calibFilename)
{
	imageSize_d = Vector2i(512, 424);
	imageSize_rgb = Vector2i(640, 480);
	
	data = new PrivateData();

	colorAvailable = false;

	HRESULT hr;

	depthAvailable = true;

	hr = GetDefaultKinectSensor(&data->kinectSensor);
	if (FAILED(hr))
	{
		depthAvailable = false;
		printf("Kinect2: Failed to initialise depth camera\n");
		return;
	}

	if (data->kinectSensor)
	{
		IDepthFrameSource* pDepthFrameSource = NULL;

		hr = data->kinectSensor->Open();

		if (SUCCEEDED(hr))
			hr = data->kinectSensor->get_DepthFrameSource(&pDepthFrameSource);

		if (SUCCEEDED(hr))
			hr = pDepthFrameSource->OpenReader(&data->depthFrameReader);

		SafeRelease(pDepthFrameSource);
	}

	if (!data->kinectSensor || FAILED(hr))
	{
		depthAvailable = false;
		printf("Kinect2: No ready Kinect 2 sensor found\n");
		return;
	}
}

Kinect2Engine::~Kinect2Engine()
{
	SafeRelease(data->depthFrameReader);

	if (data->kinectSensor) data->kinectSensor->Close();

	SafeRelease(data->kinectSensor);
}

void Kinect2Engine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
	//Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
	//if (colorAvailable)
	//{
	//}
	//else memset(rgb, 0, rgbImage->dataSize * sizeof(Vector4u));

	//float *depth = out->depth->GetData(MEMORYDEVICE_CPU);
	//if (depthAvailable)
	//{
	//	IDepthFrame* pDepthFrame = NULL;
	//	UINT16 *pBuffer = NULL;
	//	UINT nBufferSize = 0;

	//	HRESULT hr = data->depthFrameReader->AcquireLatestFrame(&pDepthFrame);

	//	if (SUCCEEDED(hr))
	//	{
	//		if (SUCCEEDED(hr))
	//			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);

	//		if (SUCCEEDED(hr))
	//		{
	//			for (int i = 0; i < imageSize_d.x * imageSize_d.y; i++)
	//			{
	//				ushort depthPix = pBuffer[i];
	//				depth[i] = depthPix == 0 ? -1.0f : (float)depthPix / 1000.0f;
	//			}
	//		}
	//	}

	//	SafeRelease(pDepthFrame);
	//}
	//else memset(depth, 0, out->depth->dataSize * sizeof(short));

	//out->inputImageType = ITMView::InfiniTAM_FLOAT_DEPTH_IMAGE;

	return /*true*/;
}

bool Kinect2Engine::hasMoreImages(void) const { return true; }
Vector2i Kinect2Engine::getDepthImageSize(void) const { return imageSize_d; }
Vector2i Kinect2Engine::getRGBImageSize(void) const { return imageSize_rgb; }

#else

using namespace InputSource;

Kinect2Engine::Kinect2Engine(const char *calibFilename) : BaseImageSourceEngine(calibFilename)
{
	printf("compiled without Kinect 2 support\n");
}
Kinect2Engine::~Kinect2Engine()
{}
void Kinect2Engine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{ return; }
bool Kinect2Engine::hasMoreImages(void) const
{ return false; }
Vector2i Kinect2Engine::getDepthImageSize(void) const
{ return Vector2i(0,0); }
Vector2i Kinect2Engine::getRGBImageSize(void) const
{ return Vector2i(0,0); }

#endif

