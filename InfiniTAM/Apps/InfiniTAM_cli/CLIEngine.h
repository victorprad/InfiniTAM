// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../InputSource/ImageSourceEngine.h"
#include "../../InputSource/IMUSourceEngine.h"
#include "../../ITMLib/Core/ITMMainEngine.h"
#include "../../ITMLib/Utils/ITMLibSettings.h"
#include "../../ORUtils/FileUtils.h"
#include "../../ORUtils/NVTimer.h"

namespace InfiniTAM
{
	namespace Engine
	{
		class CLIEngine
		{
			static CLIEngine* instance;

			InputSource::ImageSourceEngine *imageSource;
			InputSource::IMUSourceEngine *imuSource;
			ITMLib::ITMLibSettings internalSettings;
			ITMLib::ITMMainEngine *mainEngine;

			StopWatchInterface *timer_instant;
			StopWatchInterface *timer_average;

		private:
			ITMUChar4Image *inputRGBImage; ITMShortImage *inputRawDepthImage;
			ITMLib::ITMIMUMeasurement *inputIMUMeasurement;

			int currentFrameNo;
		public:
			static CLIEngine* Instance(void) {
				if (instance == NULL) instance = new CLIEngine();
				return instance;
			}

			float processedTime;

			void Initialise(InputSource::ImageSourceEngine *imageSource, InputSource::IMUSourceEngine *imuSource, ITMLib::ITMMainEngine *mainEngine,
				ITMLib::ITMLibSettings::DeviceType deviceType);
			void Shutdown();

			void Run();
			bool ProcessFrame();
		};
	}
}
