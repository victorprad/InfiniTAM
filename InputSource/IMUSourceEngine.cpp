// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "IMUSourceEngine.h"

#include "../ORUtils/FileUtils.h"

#include <stdio.h>

using namespace InputSource;
using namespace ITMLib;

IMUSourceEngine::IMUSourceEngine(const char *imuMask)
{
	strncpy(this->imuMask, imuMask, BUF_SIZE);

	currentFrameNo = 0;
	cachedFrameNo = -1;

	cached_imu = NULL;
}

void IMUSourceEngine::loadIMUIntoCache(void)
{
	char str[2048]; FILE *f; bool success = false;

	cached_imu = new ITMIMUMeasurement();

	sprintf(str, imuMask, currentFrameNo);
	f = fopen(str, "r");
	if (f)
	{
		size_t ret = fscanf(f, "%f %f %f %f %f %f %f %f %f",
			&cached_imu->R.m00, &cached_imu->R.m01, &cached_imu->R.m02,
			&cached_imu->R.m10, &cached_imu->R.m11, &cached_imu->R.m12,
			&cached_imu->R.m20, &cached_imu->R.m21, &cached_imu->R.m22);

		fclose(f);

		if (ret == 9) success = true;
	}

	if (!success) {
		delete cached_imu; cached_imu = NULL;
		printf("error reading file '%s'\n", str);
	}
}

bool IMUSourceEngine::hasMoreMeasurements(void)
{
	loadIMUIntoCache();

	return (cached_imu != NULL);
}

void IMUSourceEngine::getMeasurement(ITMIMUMeasurement *imu)
{
	bool bUsedCache = false;
	
	if (cached_imu != NULL)
	{
		imu->R = cached_imu->R;
		delete cached_imu;
		cached_imu = NULL;
		bUsedCache = true;
	}

	if (!bUsedCache) this->loadIMUIntoCache();

	++currentFrameNo;
}
