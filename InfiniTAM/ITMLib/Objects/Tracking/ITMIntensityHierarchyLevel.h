// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "TrackerIterationType.h"
#include "../../Utils/ITMMath.h"
#include "../../../ORUtils/Image.h"

namespace ITMLib
{
	class ITMIntensityHierarchyLevel
	{
	public:
		int levelId;

		TrackerIterationType iterationType;

		ORUtils::Image<float> *intensity_current;
		ORUtils::Image<float> *intensity_prev;
		ORUtils::Image<Vector2f> *gradients;
		Vector4f intrinsics;

		bool manageData;

		ITMIntensityHierarchyLevel(Vector2i imgSize, int levelId, TrackerIterationType iterationType,
			MemoryDeviceType memoryType, bool skipAllocation = false)
		{
			this->manageData = !skipAllocation;
			this->levelId = levelId;
			this->iterationType = iterationType;

			if (!skipAllocation)
			{
				this->intensity_current = new ORUtils::Image<float>(imgSize, memoryType);
				this->intensity_prev = new ORUtils::Image<float>(imgSize, memoryType);
			}

			this->gradients = new ORUtils::Image<Vector2f>(imgSize, memoryType);
		}

		void UpdateHostFromDevice()
		{ 
			this->intensity_current->UpdateHostFromDevice();
			this->intensity_prev->UpdateHostFromDevice();
			this->gradients->UpdateHostFromDevice();
		}

		void UpdateDeviceFromHost()
		{ 
			this->intensity_current->UpdateDeviceFromHost();
			this->intensity_prev->UpdateDeviceFromHost();
			this->gradients->UpdateDeviceFromHost();
		}

		~ITMIntensityHierarchyLevel(void)
		{
			if (manageData)
			{
				delete intensity_current;
				delete intensity_prev;
			}

			delete gradients;
		}

		// Suppress the default copy constructor and assignment operator
		ITMIntensityHierarchyLevel(const ITMIntensityHierarchyLevel&);
		ITMIntensityHierarchyLevel& operator=(const ITMIntensityHierarchyLevel&);
	};
}
