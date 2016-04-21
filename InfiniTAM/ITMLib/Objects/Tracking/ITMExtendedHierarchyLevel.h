// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "TrackerIterationType.h"
#include "../../Utils/ITMMath.h"
#include "../../../ORUtils/Image.h"

namespace ITMLib
{
	class ITMExtendHierarchyLevel
	{
	public:
		int levelId;

		TrackerIterationType iterationType;

		ORUtils::Image<float> *depth;

		Vector4f intrinsics;
		bool manageData;

		ITMExtendHierarchyLevel(Vector2i imgSize, int levelId, TrackerIterationType iterationType, 
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

		~ITMExtendHierarchyLevel(void)
		{
			if (manageData)
			{
				delete depth;
			}
		}

		// Suppress the default copy constructor and assignment operator
		ITMExtendHierarchyLevel(const ITMExtendHierarchyLevel&);
		ITMExtendHierarchyLevel& operator=(const ITMExtendHierarchyLevel&);
	};
}
