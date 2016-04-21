// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include <stdlib.h>

#include "../../../InputSource/IMUSourceEngine.h"
#include "../../../InputSource/ImageSourceEngine.h"
#include "../../../ITMLib/Core/ITMMainEngine.h"
#include "../../../ORUtils/NVTimer.h"

#include "../../../InputSource/FFMPEGWriter.h"

class InfiniTAMApp {
	public:
	static InfiniTAMApp* Instance(void)
	{
		if (globalInstance==NULL) globalInstance = new InfiniTAMApp();
		return globalInstance;
	}

	InfiniTAMApp(void);
	~InfiniTAMApp(void);

	void InitGL();
	void ResizeGL(int newWidth, int newHeight);
	void RenderGL(void);

	void StartProcessing(int useLiveCamera);
	bool ProcessFrame(void);
	void StopProcessing(void);

	bool IsInitialized(void) const
	{ return mIsInitialized; }

	void toggleRecordingMode(void);

	float getAverageTime(void);

	private:
	static InfiniTAMApp *globalInstance;

	InfiniTAM::Engine::ImageSourceEngine *mImageSource;
	InfiniTAM::Engine::IMUSourceEngine *mImuSource;
	ITMLib::ITMLibSettings *mInternalSettings;
	ITMLib::ITMMainEngine *mMainEngine;

	ITMUChar4Image *inputRGBImage; ITMShortImage *inputRawDepthImage;
	ITMLib::ITMIMUMeasurement *inputIMUMeasurement;

	StopWatchInterface *timer_instant;
	StopWatchInterface *timer_average;

	static const int NUM_WIN = 3;
	Vector4f winPos[NUM_WIN];
	uint textureId[NUM_WIN];
	ITMLib::ITMMainEngine::GetImageType winImageType[NUM_WIN];
	ITMUChar4Image *outImage[NUM_WIN];

	Vector2i mNewWindowSize;
	bool mIsInitialized;
	bool mRecordingMode;

	InfiniTAM::FFMPEGWriter *depthVideoWriter;
	InfiniTAM::FFMPEGWriter *colorVideoWriter;
};


