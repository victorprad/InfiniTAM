// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "Kinect2Engine.h"

#include "../Utils/FileUtils.h"

#include <stdio.h>

#ifndef COMPILE_WITHOUT_Kinect2
#include <Kinect.h>

#pragma comment(lib, "kinect20.lib")

using namespace InfiniTAM::Engine;

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
	IColorFrameReader* colorFrameReader;
};

Kinect2Engine::Kinect2Engine(const char *calibFilename) : ImageSourceEngine(calibFilename)
{
	imageSize_d = Vector2i(cDepthWidth, cDepthHeight);
	imageSize_rgb = Vector2i(cColorWidth, cColorHeight);

	data = new PrivateData();

	colorAvailable = true;

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

		IColorFrameSource* pColorFrameSource = NULL;

		if (SUCCEEDED(hr))
			hr = data->kinectSensor->get_ColorFrameSource(&pColorFrameSource);

		if (SUCCEEDED(hr))
			hr = pColorFrameSource->OpenReader(&data->colorFrameReader);

		SafeRelease(pColorFrameSource);
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
	SafeRelease(data->colorFrameReader);

	if (data->kinectSensor) data->kinectSensor->Close();

	SafeRelease(data->kinectSensor);
}

void Kinect2Engine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
	Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);

	if (colorAvailable)
	{
		IFrameDescription* pFrameDescription = NULL;
		int nWidth = 0;
		int nHeight = 0;
		IColorFrame* pColorFrame = NULL;
		RGBQUAD *pBuffer = NULL;
		UINT nBufferSize = 0;

		HRESULT hr = data->colorFrameReader->AcquireLatestFrame(&pColorFrame);

		// Get frame descriptor to get the width and the height of the image frame.
		if (SUCCEEDED(hr))
			hr = pColorFrame->get_FrameDescription(&pFrameDescription);

		if (SUCCEEDED(hr))
			hr = pFrameDescription->get_Width(&nWidth);

		if (SUCCEEDED(hr))
			hr = pFrameDescription->get_Height(&nHeight);

		SafeRelease(pFrameDescription);

		RGBQUAD* m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];

		if (SUCCEEDED(hr))
		{
			// Must first determine the color image format
			ColorImageFormat imageFormat = ColorImageFormat_None;
			hr = pColorFrame->get_RawColorImageFormat(&imageFormat);

			if (imageFormat == ColorImageFormat_Bgra)
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&pBuffer));
			}
			else if (m_pColorRGBX)
			{
				pBuffer = m_pColorRGBX;
				nBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);
			}
			else
			{
				hr = E_FAIL;
			}

			if (SUCCEEDED(hr)) {
				// Double check width and height of the frame
				if (pBuffer && (nWidth == cColorWidth) && (nHeight == cColorHeight))
				{
					// Copy the received frame to rgbImage
					for (int i = 0; i < rgbImage->noDims.x * rgbImage->noDims.y; i++)
					{
						Vector4u newPix;
						newPix.x = pBuffer[i].rgbRed; newPix.y = pBuffer[i].rgbGreen; newPix.z = pBuffer[i].rgbBlue; newPix.w = 255;
						rgb[i] = newPix;
					}
				}
			}
		}

		if (m_pColorRGBX)
			delete[] m_pColorRGBX;

		// If you don't release the color frame, you won't be able to get the next frame
		SafeRelease(pColorFrame);

	}
	else memset(rgb, 0, rgbImage->dataSize * sizeof(Vector4u));

	// Now process the depth frame
	short *depth = rawDepthImage->GetData(MEMORYDEVICE_CPU);
	if (depthAvailable)
	{
		IFrameDescription* pFrameDescription = NULL;
		int nWidth = 0;
		int nHeight = 0;
		USHORT nDepthMinReliableDistance = 0;
		USHORT nDepthMaxReliableDistance = 0;
		IDepthFrame* pDepthFrame = NULL;
		UINT16 *pBuffer = NULL;
		UINT nBufferSize = 0;

		HRESULT hr = data->depthFrameReader->AcquireLatestFrame(&pDepthFrame);

		// Get frame descriptor to get the width and the height of the image frame.
		if (SUCCEEDED(hr))
			hr = pDepthFrame->get_FrameDescription(&pFrameDescription);

		if (SUCCEEDED(hr))
			hr = pFrameDescription->get_Width(&nWidth);

		if (SUCCEEDED(hr))
			hr = pFrameDescription->get_Height(&nHeight);

		SafeRelease(pFrameDescription);

		// Get the minimum and maximum reliable distances.
		if (SUCCEEDED(hr))
			hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);

		if (SUCCEEDED(hr))
			hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);

			if (SUCCEEDED(hr) && pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight)) {

				for (int i = 0; i < imageSize_d.x * imageSize_d.y; i++)
				{
					ushort depthPix = pBuffer[i];

					// Filter depth value based on min and max reliable distances.
					if (depthPix < nDepthMinReliableDistance)
						depthPix = 0;
					else if (depthPix > nDepthMaxReliableDistance)
						depthPix = nDepthMaxReliableDistance;

					depth[i] = depthPix;
				}

				SafeRelease(pDepthFrame);
			}
		}
	}
	else memset(depth, 0, rawDepthImage->dataSize * sizeof(short));
}

bool Kinect2Engine::hasMoreImages(void) { return true; }
Vector2i Kinect2Engine::getDepthImageSize(void) { return imageSize_d; }
Vector2i Kinect2Engine::getRGBImageSize(void) { return imageSize_rgb; }

#else

using namespace InfiniTAM::Engine;

Kinect2Engine::Kinect2Engine(const char *calibFilename) : ImageSourceEngine(calibFilename)
{
	printf("compiled without Kinect 2 support\n");
}
Kinect2Engine::~Kinect2Engine()
{}
void Kinect2Engine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
	return;
}
bool Kinect2Engine::hasMoreImages(void)
{
	return false;
}
Vector2i Kinect2Engine::getDepthImageSize(void)
{
	return Vector2i(0, 0);
}
Vector2i Kinect2Engine::getRGBImageSize(void)
{
	return Vector2i(0, 0);
}

#endif

