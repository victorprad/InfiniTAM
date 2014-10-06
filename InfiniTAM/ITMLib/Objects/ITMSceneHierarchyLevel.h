// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Objects/ITMImage.h"

namespace ITMLib
{
	namespace Objects
	{
		class ITMSceneHierarchyLevel
		{
		public:
			int levelId;

			bool rotationOnly;

			ITMFloat4Image *pointsMap;
			ITMFloat4Image *normalsMap;
			Vector4f intrinsics;

			ITMSceneHierarchyLevel(Vector2i imgSize, int levelId, bool rotationOnly, bool useGPU)
			{
				this->levelId = levelId;
				this->rotationOnly = rotationOnly;

				this->pointsMap = new ITMFloat4Image(imgSize, useGPU);
				this->normalsMap = new ITMFloat4Image(imgSize, useGPU);
			}

			void UpdateHostFromDevice()
			{ 
				this->pointsMap->UpdateHostFromDevice();
				this->normalsMap->UpdateHostFromDevice();
			}

			void UpdateDeviceFromHost()
			{ 
				this->pointsMap->UpdateDeviceFromHost();
				this->normalsMap->UpdateDeviceFromHost();
			}

			~ITMSceneHierarchyLevel(void)
			{
				delete pointsMap;
				delete normalsMap;
			}

			// Suppress the default copy constructor and assignment operator
			ITMSceneHierarchyLevel(const ITMSceneHierarchyLevel&);
			ITMSceneHierarchyLevel& operator=(const ITMSceneHierarchyLevel&);
		};
	}
}
