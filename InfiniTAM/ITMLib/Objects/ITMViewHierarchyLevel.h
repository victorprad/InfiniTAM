// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

namespace ITMLib
{
	namespace Objects
	{
		class ITMViewHierarchyLevel
		{
		public:
			int levelId;

			TrackerIterationType iterationType;

			ITMUChar4Image *rgb; ITMFloatImage *depth;
			ITMShort4Image *gradientX_rgb, *gradientY_rgb;
			Vector4f intrinsics;

			bool manageData;

			ITMViewHierarchyLevel(Vector2i imgSize, int levelId, TrackerIterationType iterationType, MemoryDeviceType memoryType, bool skipAllocation)
			{
				this->manageData = !skipAllocation;
				this->levelId = levelId;
				this->iterationType = iterationType;

				if (!skipAllocation) {
					this->rgb = new ITMUChar4Image(imgSize, memoryType);
					this->depth = new ITMFloatImage(imgSize, memoryType);
					this->gradientX_rgb = new ITMShort4Image(imgSize, memoryType);
					this->gradientY_rgb = new ITMShort4Image(imgSize, memoryType);
				}
			}

			void UpdateHostFromDevice()
			{ 
				this->rgb->UpdateHostFromDevice();
				this->depth->UpdateHostFromDevice();
				this->gradientX_rgb->UpdateHostFromDevice();
				this->gradientY_rgb->UpdateHostFromDevice();
			}

			void UpdateDeviceFromHost()
			{ 
				this->rgb->UpdateDeviceFromHost();
				this->depth->UpdateHostFromDevice();
				this->gradientX_rgb->UpdateDeviceFromHost();
				this->gradientY_rgb->UpdateDeviceFromHost();
			}

			~ITMViewHierarchyLevel(void)
			{
				if (manageData) {
					delete rgb;
					delete depth;
					delete gradientX_rgb; delete gradientY_rgb;
				}
			}

			// Suppress the default copy constructor and assignment operator
			ITMViewHierarchyLevel(const ITMViewHierarchyLevel&);
			ITMViewHierarchyLevel& operator=(const ITMViewHierarchyLevel&);
		};
	}
}
