// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include <stdlib.h>

namespace ITMLib
{
	namespace Objects
	{
		class ITMPointCloud
		{
		public:
			uint noTotalPoints;

			ITMFloat4Image *locations, *colours;

			explicit ITMPointCloud(Vector2i imgSize, bool useGPU)
			{
				this->noTotalPoints = 0;

				locations = new ITMFloat4Image(imgSize, useGPU);
				colours = new ITMFloat4Image(imgSize, useGPU);
			}

			void UpdateHostFromDevice()
			{
				this->locations->UpdateHostFromDevice();
				this->colours->UpdateHostFromDevice();
			}

			void UpdateDeviceFromHost()
			{
				this->locations->UpdateDeviceFromHost();
				this->colours->UpdateDeviceFromHost();
			}

			~ITMPointCloud()
			{
				delete locations;
				delete colours;
			}

			// Suppress the default copy constructor and assignment operator
			ITMPointCloud(const ITMPointCloud&);
			ITMPointCloud& operator=(const ITMPointCloud&);
		};
	}
}
