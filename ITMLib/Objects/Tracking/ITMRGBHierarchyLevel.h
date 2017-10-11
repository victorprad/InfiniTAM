// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "TrackerIterationType.h"
#include "../../Utils/ITMMath.h"
#include "../../../ORUtils/Image.h"

namespace ITMLib
{
	class ITMRGBHierarchyLevel
	{
	public:
		int levelId;

		TrackerIterationType iterationType;

		ORUtils::Image<Vector4u> *rgb_current;

		ORUtils::Image<Vector4u> *rgb_prev;
		ORUtils::Image<Vector4s> *gX, *gY;
		Vector4f intrinsics;

		bool manageData;

		ITMRGBHierarchyLevel(Vector2i imgSize, int levelId, TrackerIterationType iterationType,
			MemoryDeviceType memoryType, bool skipAllocation = false)
		{
			this->manageData = !skipAllocation;
			this->levelId = levelId;
			this->iterationType = iterationType;

			if (!skipAllocation)
			{
				this->rgb_current = new ORUtils::Image<Vector4u>(imgSize, memoryType);
				this->rgb_prev = new ORUtils::Image<Vector4u>(imgSize, memoryType);
			}

			this->gX = new ORUtils::Image<Vector4s>(imgSize, memoryType);
			this->gY = new ORUtils::Image<Vector4s>(imgSize, memoryType);
		}

		void UpdateHostFromDevice()
		{ 
			this->rgb_current->UpdateHostFromDevice();
			this->rgb_prev->UpdateHostFromDevice();
			this->gX->UpdateHostFromDevice();
			this->gY->UpdateHostFromDevice();
		}

		void UpdateDeviceFromHost()
		{ 
			this->rgb_current->UpdateDeviceFromHost();
			this->rgb_prev->UpdateDeviceFromHost();
			this->gX->UpdateDeviceFromHost();
			this->gY->UpdateDeviceFromHost();
		}

		~ITMRGBHierarchyLevel(void)
		{
			if (manageData)
			{
				delete rgb_current;
				delete rgb_prev;
			}

			delete gX;
			delete gY;
		}

		// Suppress the default copy constructor and assignment operator
		ITMRGBHierarchyLevel(const ITMRGBHierarchyLevel&);
		ITMRGBHierarchyLevel& operator=(const ITMRGBHierarchyLevel&);
	};
}
