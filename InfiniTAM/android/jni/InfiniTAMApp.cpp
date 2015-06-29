// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "InfiniTAMApp.h"
#include "../../Engine/OpenNIEngine.h"
#include "../../Engine/IMUSourceEngine.h"

#include <GLES/gl.h>

#include <android/log.h>
#include <unistd.h>

InfiniTAMApp* InfiniTAMApp::globalInstance = NULL;

InfiniTAMApp::InfiniTAMApp(void)
{
	mImageSource = NULL;
	mImuSource = NULL;
	mMainEngine = NULL;
	mIsInitialized = false;

	winImageType[0] = ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST;
	winImageType[1] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH;
	winImageType[2] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB;
}

void InfiniTAMApp::InitGL(void)
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glGenTextures(NUM_WIN, textureId);
}

void InfiniTAMApp::ResizeGL(int newWidth, int newHeight)
{
	mNewWindowSize.x = newWidth;
	mNewWindowSize.y = newHeight;
}

void InfiniTAMApp::RenderGL(void)
{
	glClear(GL_COLOR_BUFFER_BIT);
	if (!IsInitialized()) return;

	if (mNewWindowSize.x > 0) {
		Vector2i depthImageSize = mMainEngine->GetImageSize();
		float ratio = (float)depthImageSize.x/(float)depthImageSize.y;

//		glViewport(0, 0, newWidth, newHeight);
		if (mNewWindowSize.x >= mNewWindowSize.y) {
			winPos[0] = Vector4f(0.0f, 0.0f, 1.0f/ratio, 1.0f);
			winPos[1] = Vector4f(1.0f/ratio, 0.5f, 1.0f, 1.0f);
			winPos[2] = Vector4f(1.0f/ratio, 0.0f, 1.0f, 0.5f);
		} else {
			winPos[0] = Vector4f(0.0f, 1.0f/3.0f, 1.0f, 1.0f);
			winPos[1] = Vector4f(0.0f, 0.0f, 0.5f, 1.0f/3.0f);
			winPos[2] = Vector4f(0.5f, 0.0f, 1.0f, 1.0f/3.0f);
		}

		mNewWindowSize.x = mNewWindowSize.y = -1;
	}

	int localNumWin = 1;//NUM_WIN
	for (int w = 0; w < localNumWin; w++) mMainEngine->GetImage(outImage[w], winImageType[w]);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	{
		glLoadIdentity();
		glOrthof(0.0f, 1.0f, 0.0f, 1.0f, -1.0f, 1.0f);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		{
			glEnable(GL_TEXTURE_2D);
			glDisable(GL_BLEND);
			glEnableClientState(GL_VERTEX_ARRAY);
			glEnableClientState(GL_TEXTURE_COORD_ARRAY);
			for (int w = 0; w < localNumWin; w++) {
				glBindTexture(GL_TEXTURE_2D, textureId[w]);
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, outImage[w]->noDims.x, outImage[w]->noDims.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, outImage[w]->GetData(MEMORYDEVICE_CPU));
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

				float vertices[] = {
					winPos[w][0], winPos[w][3],
					winPos[w][2], winPos[w][3],
					winPos[w][0], winPos[w][1],
					winPos[w][2], winPos[w][1] };
				float texture[] = {
					0.0f, 0.0f,
					1.0f, 0.0f,
					0.0f, 1.0f,
					1.0f, 1.0f };

				glVertexPointer(2, GL_FLOAT, 0, vertices);
				glTexCoordPointer(2, GL_FLOAT, 0, texture);
				glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
			}
			glDisableClientState(GL_VERTEX_ARRAY);
			glDisableClientState(GL_TEXTURE_COORD_ARRAY);
			glDisable(GL_TEXTURE_2D);
		}
		glPopMatrix();
	}
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
}

void InfiniTAMApp::StartProcessing(int useLiveCamera)
{
	const char *calibFile = "/storage/sdcard0/InfiniTAM/teddy/calib.txt";
	const char *imagesource_part1 = "/storage/sdcard0/InfiniTAM/teddy/video%06i.ppm";
	const char *imagesource_part2 = "/storage/sdcard0/InfiniTAM/teddy/depth%06i.ppm";
/*	const char *calibFile = "/storage/sdcard0/InfiniTAM/CAsmall/calib.txt";
	const char *imagesource_part1 = "/storage/sdcard0/InfiniTAM/CAsmall/img_0000%04i.irw";
	const char *imagesource_part2 = "/storage/sdcard0/InfiniTAM/CAsmall/img_0000%04i.irw";
	const char *imagesource_part3 = "/storage/sdcard0/InfiniTAM/CAsmall/imu_0000%04i.txt";*/

	mInternalSettings = new ITMLibSettings();
	mImuSource = NULL; //new IMUSourceEngine;
	if (useLiveCamera == 0) {
		mImageSource = new InfiniTAM::Engine::ImageFileReader(calibFile, imagesource_part1, imagesource_part2);
		//mImageSource = new InfiniTAM::Engine::RawFileReader(calibFile, imagesource_part1, imagesource_part2, Vector2i(320, 240), 0.5f);
		//mImuSource = new InfiniTAM::Engine::IMUSourceEngine(imagesource_part3);
		//mImageSource = new InfiniTAM::Engine::OpenNIEngine(calibFile, "/storage/sdcard0/InfiniTAM/50Hz_closeup.oni");
	} else {
		mImageSource = new InfiniTAM::Engine::OpenNIEngine(calibFile);
	}
	mMainEngine = new ITMMainEngine(mInternalSettings, &mImageSource->calib, mImageSource->getRGBImageSize(), mImageSource->getDepthImageSize());

	bool allocateGPU = false;
	if (mInternalSettings->deviceType == ITMLibSettings::DEVICE_CUDA) allocateGPU = true;

	for (int w = 0; w < NUM_WIN; w++) {
		outImage[w] = new ITMUChar4Image(mImageSource->getDepthImageSize(), true, allocateGPU);
	}

	inputRGBImage = new ITMUChar4Image(mImageSource->getRGBImageSize(), true, allocateGPU);
	inputRawDepthImage = new ITMShortImage(mImageSource->getDepthImageSize(), true, allocateGPU);
	inputIMUMeasurement = new ITMIMUMeasurement();

#ifndef COMPILE_WITHOUT_CUDA
	ITMSafeCall(cudaThreadSynchronize());
#endif

	sdkCreateTimer(&timer_instant);
	sdkCreateTimer(&timer_average);
	sdkResetTimer(&timer_average);

	mIsInitialized = true;
}

bool InfiniTAMApp::ProcessFrame(void)
{
	if (!mImageSource->hasMoreImages()) return false;
	mImageSource->getImages(inputRGBImage, inputRawDepthImage);

	if (mImuSource != NULL) {
		if (!mImuSource->hasMoreMeasurements()) return false;
		else mImuSource->getMeasurement(inputIMUMeasurement);
	}

	sdkResetTimer(&timer_instant);
	sdkStartTimer(&timer_instant); sdkStartTimer(&timer_average);

	//actual processing on the mainEngine
	if (mImuSource != NULL) mMainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, inputIMUMeasurement);
	else mMainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage);

	ITMSafeCall(cudaDeviceSynchronize());
	sdkStopTimer(&timer_instant); sdkStopTimer(&timer_average);

	__android_log_print(ANDROID_LOG_VERBOSE, "InfiniTAM", "Process Frame finished: %f %f", sdkGetTimerValue(&timer_instant), sdkGetAverageTimerValue(&timer_average));

	return true;
}

