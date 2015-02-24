#include "InfiniTAMApp.h"

#include <GLES/gl.h>
//#include <GLES2/gl2.h>

#include <android/log.h>

InfiniTAMApp* InfiniTAMApp::globalInstance = NULL;//new InfiniTAMApp();

InfiniTAMApp::InfiniTAMApp(void)
{
	const char *calibFile = "/storage/sdcard0/InfiniTAM/teddy/calib.txt";
	const char *imagesource_part1 = "/storage/sdcard0/InfiniTAM/teddy/video%06i.ppm";
	const char *imagesource_part2 = "/storage/sdcard0/InfiniTAM/teddy/depth%06i.ppm";

	mInternalSettings = new ITMLibSettings();
	mImageSource = new InfiniTAM::Engine::ImageFileReader(calibFile, imagesource_part1, imagesource_part2);//ImageSourceEngine;
	mImuSource = NULL; //new IMUSourceEngine;
	mMainEngine = new ITMMainEngine(mInternalSettings, &mImageSource->calib, mImageSource->getRGBImageSize(), mImageSource->getDepthImageSize());

	winImageType[0] = ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST;
	winImageType[1] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH;
	winImageType[2] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB;

__android_log_print(ANDROID_LOG_VERBOSE, "InfiniTAM", "constructor. imgSize %i %i, %i %i", mImageSource->getDepthImageSize().x, mImageSource->getDepthImageSize().y, mImageSource->getRGBImageSize().x, mImageSource->getRGBImageSize().y);

	bool allocateGPU = false;
	if (mInternalSettings->deviceType == ITMLibSettings::DEVICE_CUDA) allocateGPU = true;

	for (int w = 0; w < NUM_WIN; w++) {
		outImage[w] = new ITMUChar4Image(mImageSource->getDepthImageSize(), true, allocateGPU);
	}

	inputRGBImage = new ITMUChar4Image(mImageSource->getRGBImageSize(), true, allocateGPU);
	inputRawDepthImage = new ITMShortImage(mImageSource->getDepthImageSize(), true, allocateGPU);
	inputIMUMeasurement = new ITMIMUMeasurement();

	sdkCreateTimer(&timer_instant);
	sdkCreateTimer(&timer_average);
	sdkResetTimer(&timer_average);
}

void InfiniTAMApp::InitGL(void)
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glGenTextures(NUM_WIN, textureId);
__android_log_print(ANDROID_LOG_VERBOSE, "InfiniTAM", "InitGL");
}

void InfiniTAMApp::ResizeGL(int newWidth, int newHeight)
{
__android_log_print(ANDROID_LOG_VERBOSE, "InfiniTAM", "ResizeGL: %i %i", newWidth, newHeight);
	Vector2i depthImageSize = mImageSource->getDepthImageSize();
	float ratio = (float)depthImageSize.x/(float)depthImageSize.y;

	glViewport(0, 0, newWidth, newHeight);
	if (newWidth >= newHeight) {
		winPos[0] = Vector4f(0.0f, 0.0f, 1.0f/ratio, 1.0f);
		winPos[1] = Vector4f(1.0f/ratio, 0.5f, 1.0f, 1.0f);
		winPos[2] = Vector4f(1.0f/ratio, 0.0f, 1.0f, 0.5f);
	} else {
		winPos[0] = Vector4f(0.0f, 1.0f/3.0f, 1.0f, 1.0f);
		winPos[1] = Vector4f(0.0f, 0.0f, 0.5f, 1.0f/3.0f);
		winPos[2] = Vector4f(0.5f, 0.0f, 1.0f, 1.0f/3.0f);
	}
}

void InfiniTAMApp::RenderGL(void)
{
__android_log_print(ANDROID_LOG_VERBOSE, "InfiniTAM", "RenderGL");
	glClear(GL_COLOR_BUFFER_BIT);
//	glColor3f(1.0f, 1.0f, 1.0f);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	{
		glLoadIdentity();
		glOrthof(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		{
			glEnable(GL_TEXTURE_2D);
			glEnableClientState(GL_VERTEX_ARRAY);
			glEnableClientState(GL_TEXTURE_COORD_ARRAY);
			for (int w = 0; w < NUM_WIN; w++) {

				glBindTexture(GL_TEXTURE_2D, textureId[w]);
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, outImage[w]->noDims.x, outImage[w]->noDims.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, outImage[w]->GetData(MEMORYDEVICE_CPU));
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

				float vertices[] = {
					winPos[w][0], winPos[w][1],
					winPos[w][2], winPos[w][1],
					winPos[w][0], winPos[w][3],
					winPos[w][2], winPos[w][3] };
				float texture[] = {
					0.0f, 0.0f,
					1.0f, 0.0f,
					0.0f, 1.0f,
					1.0f, 1.0f };
				unsigned char indices[] = {
					0, 1, 3,
					0, 3, 2 };

				glVertexPointer(2, GL_FLOAT, 0, vertices);
				glTexCoordPointer(2, GL_FLOAT, 0, texture);
				glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_BYTE, indices);

			}
			glDisableClientState(GL_VERTEX_ARRAY);
			glDisableClientState(GL_TEXTURE_COORD_ARRAY);
			glDisable(GL_TEXTURE_2D);
		}
		glPopMatrix();
	}
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glFlush();
}

bool InfiniTAMApp::ProcessFrame(void)
{
__android_log_print(ANDROID_LOG_VERBOSE, "InfiniTAM", "Process Frame");
	if (!mImageSource->hasMoreImages()) return false;
	mImageSource->getImages(inputRGBImage, inputRawDepthImage);

	if (mImuSource != NULL) {
		if (!mImuSource->hasMoreMeasurements()) return false;
		else mImuSource->getMeasurement(inputIMUMeasurement);
	}

	sdkResetTimer(&timer_instant);
	sdkStartTimer(&timer_instant); sdkStartTimer(&timer_average);

	//actual processing on the mailEngine
	if (mImuSource != NULL) mMainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, inputIMUMeasurement);
	else mMainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage);

	sdkStopTimer(&timer_instant); sdkStopTimer(&timer_average);
__android_log_print(ANDROID_LOG_VERBOSE, "InfiniTAM", "Process Frame finished: %f %f", sdkGetTimerValue(&timer_instant), sdkGetAverageTimerValue(&timer_average));

	return true;
}

