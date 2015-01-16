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

			ITMImage<Vector4f> *locations, *colours;

			explicit ITMPointCloud(Vector2i imgSize, MemoryDeviceType memoryType)
			{
				this->noTotalPoints = 0;

				locations = new ITMImage<Vector4f>(imgSize, memoryType);
				colours = new ITMImage<Vector4f>(imgSize, memoryType);
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
