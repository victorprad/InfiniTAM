// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "TrackerIterationType.h"
#include "../../Utils/ITMMath.h"
#include "../../../ORUtils/Image.h"

namespace ITMLib
{
	class ITMDepthHierarchyLevel
	{
	public:
		int levelId;

		TrackerIterationType iterationType;

		ORUtils::Image<float> *depth;

		Vector4f intrinsics;
		bool manageData;

		ITMDepthHierarchyLevel(Vector2i imgSize, int levelId, TrackerIterationType iterationType,
			MemoryDeviceType memoryType, bool skipAllocation = false)
		{
			this->manageData = !skipAllocation;
			this->levelId = levelId;
			this->iterationType = iterationType;

			if (!skipAllocation)
			{
				this->depth = new ORUtils::Image<float>(imgSize, memoryType);
			}
		}

		void UpdateHostFromDevice()
		{ 
			this->depth->UpdateHostFromDevice();
		}

		void UpdateDeviceFromHost()
		{ 
			this->depth->UpdateDeviceFromHost();
		}

		~ITMDepthHierarchyLevel(void)
		{
			if (manageData)
			{
				delete depth;
			}
		}

		// Suppress the default copy constructor and assignment operator
		ITMDepthHierarchyLevel(const ITMDepthHierarchyLevel&);
		ITMDepthHierarchyLevel& operator=(const ITMDepthHierarchyLevel&);
	};
}
