// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../ITMLib/ITMLib.h"

namespace InfiniTAM
{
	namespace Engine
	{
		class IMUSourceEngine
		{
		private:
			static const int BUF_SIZE = 2048;
			char imuMask[BUF_SIZE];

			ITMIMUMeasurement *cached_imu;

			void loadIMUIntoCache();
			int cachedFrameNo;
			int currentFrameNo;

		public:
			IMUSourceEngine(const char *imuMask);
			~IMUSourceEngine() { }

			bool hasMoreMeasurements(void);
			void getMeasurement(ITMIMUMeasurement *imu);
		};
	}
}

