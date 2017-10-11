// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "TrackerIterationType.h"
#include "../../Utils/ITMMath.h"
#include "../../../ORUtils/MemoryBlock.h"

namespace ITMLib
{
	template <class ImageType>
	class ITMTemplatedHierarchyLevel
	{
	public:
		int levelId;

		TrackerIterationType iterationType;

		ImageType *data;

		Vector4f intrinsics;
		bool manageData;

		ITMTemplatedHierarchyLevel(Vector2i imgSize, int levelId, TrackerIterationType iterationType, 
			MemoryDeviceType memoryType, bool skipAllocation = false)
		{
			this->manageData = !skipAllocation;
			this->levelId = levelId;
			this->iterationType = iterationType;

			if (!skipAllocation) this->data = new ImageType(imgSize, memoryType);
		}

		void UpdateHostFromDevice()
		{ 
			this->data->UpdateHostFromDevice();
		}

		void UpdateDeviceFromHost()
		{ 
			this->data->UpdateDeviceFromHost();
		}

		~ITMTemplatedHierarchyLevel(void)
		{
			if (manageData) delete data;
		}

		// Suppress the default copy constructor and assignment operator
		ITMTemplatedHierarchyLevel(const ITMTemplatedHierarchyLevel&);
		ITMTemplatedHierarchyLevel& operator=(const ITMTemplatedHierarchyLevel&);
	};
}
