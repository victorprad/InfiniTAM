// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "Kinect2Engine.h"

#include "../Utils/FileUtils.h"

#include <stdio.h>

#ifndef COMPILE_WITHOUT_Kinect2SDK
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
	};

	Kinect2Engine::Kinect2Engine(const char *calibFilename) : ImageSourceEngine(calibFilename)
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

	void Kinect2Engine::getImages(ITMView *out)
	{
		Vector4u *rgb = out->rgb->GetData(false);
		if (colorAvailable)
		{
		}
		else memset(rgb, 0, out->rgb->dataSize * sizeof(Vector4u));

		float *depth = out->depth->GetData(false);
		if (depthAvailable)
		{
			IDepthFrame* pDepthFrame = NULL;
			UINT16 *pBuffer = NULL;
			UINT nBufferSize = 0;

			HRESULT hr = data->depthFrameReader->AcquireLatestFrame(&pDepthFrame);

			if (SUCCEEDED(hr))
			{
				if (SUCCEEDED(hr))
					hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);

				if (SUCCEEDED(hr))
				{
					for (int i = 0; i < imageSize_d.x * imageSize_d.y; i++)
					{
						ushort depthPix = pBuffer[i];
						depth[i] = depthPix == 0 ? -1.0f : (float)depthPix / 1000.0f;
					}
				}
			}

			SafeRelease(pDepthFrame);
		}
		else memset(depth, 0, out->depth->dataSize * sizeof(short));

		out->inputImageType = ITMView::InfiniTAM_FLOAT_DEPTH_IMAGE;

		return /*true*/;
	}

	bool Kinect2Engine::hasMoreImages(void) { return true; }
	Vector2i Kinect2Engine::getDepthImageSize(void) { return imageSize_d; }
	Vector2i Kinect2Engine::getRGBImageSize(void) { return imageSize_rgb; }
#else

	#ifndef COMPILE_WITHOUT_freenect2
		#include <signal.h>

		#include <opencv2/opencv.hpp>
		#include <libfreenect2/libfreenect2.hpp>
		#include <libfreenect2/frame_listener_impl.h>
		#include <libfreenect2/threading.h>
		#include <libfreenect2/registration.h>

		using namespace InfiniTAM::Engine;

		class Kinect2Engine::PrivateData {
			public:
			PrivateData(void);
			int start();
			int stop();
			libfreenect2::Freenect2 freenect2;
		  	libfreenect2::Freenect2Device *dev;
			libfreenect2::SyncMultiFrameListener* listener;
		 	libfreenect2::FrameMap frames;
		};

		Kinect2Engine::PrivateData::PrivateData(void)
		{
			dev = freenect2.openDefaultDevice();
		}

		int Kinect2Engine::PrivateData::start()
		{
			listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth);
			dev->setColorFrameListener(listener);
		 	dev->setIrAndDepthFrameListener(listener);
		  	dev->start();
			std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
			std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
			return 1;
		}

		int Kinect2Engine::PrivateData::stop()
		{
			dev->stop();
			dev->close();
			return 1;
		}

		Kinect2Engine::Kinect2Engine(const char *calibFilename) : ImageSourceEngine(calibFilename)
		{
			imageSize_d = Vector2i(512, 424);
			imageSize_rgb = Vector2i(1920, 1080);
			
			data = new PrivateData();

			colorAvailable = true;
			depthAvailable = true;

			if(data->dev == 0)
			{
				depthAvailable = false;
				colorAvailable = false;
				printf("Kinect2: No ready Kinect 2 sensor found\n");
				return;
			}

			data->start();
		}

		Kinect2Engine::~Kinect2Engine()
		{
			data->stop();
		}

		void Kinect2Engine::getImages(ITMView *out)
		{
			Vector4u *rgb = out->rgb->GetData(false);
			float *depth = out->depth->GetData(false);
			if (colorAvailable&&depthAvailable)	// in libFreenect2, both data are available or neither is available
			{
			    data->listener->waitForNewFrame(data->frames);
			    
				libfreenect2::Frame *rgbFrame = data->frames[libfreenect2::Frame::Color];
				for (int i = 0; i < imageSize_rgb.x*imageSize_rgb.y*3; i+=3)
				{
					rgb[i/3].b   = rgbFrame->data[i+0];
					rgb[i/3].g = rgbFrame->data[i+1];
					rgb[i/3].r = rgbFrame->data[i+2];
					rgb[i/3].a = 255.0;

				}

			    libfreenect2::Frame *depthFrame = data->frames[libfreenect2::Frame::Depth];
			    cv::Mat depthMat(424, 512, CV_32FC1, depthFrame->data);
			    cv::Mat depthMatT;
			    cv::transpose(depthMat, depthMatT);
			    depthMatT /= 4500.0f;
				for (int i = 0; i < imageSize_d.x; i++)
				{
					for (int j = 0; j< imageSize_d.y; j++)
					{
						depth[i*imageSize_d.x+j] = depthMatT.at<float>(j,i);
					}
					
				}
				data->listener->release(data->frames);
			}else{
				memset(depth, 0, out->depth->dataSize * sizeof(short));
				memset(rgb, 0, out->rgb->dataSize * sizeof(Vector4u));
			}

			out->inputImageType = ITMView::InfiniTAM_FLOAT_DEPTH_IMAGE;
			return /*true*/;
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
		void Kinect2Engine::getImages(ITMView *out)
		{ return; }
		bool Kinect2Engine::hasMoreImages(void)
		{ return false; }
		Vector2i Kinect2Engine::getDepthImageSize(void)
		{ return Vector2i(0,0); }
		Vector2i Kinect2Engine::getRGBImageSize(void)
		{ return Vector2i(0,0); }

	#endif
#endif