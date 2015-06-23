// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

namespace ITMLib
{
	namespace Objects
	{
		template <class ImageType>
		class ITMTemplatedHierarchyLevel
		{
		public:
			int levelId;

			TrackerIterationType iterationType;

			ImageType *depth;
			Vector4f intrinsics;
			bool manageData;

			ITMTemplatedHierarchyLevel(Vector2i imgSize, int levelId, TrackerIterationType iterationType, 
				MemoryDeviceType memoryType, bool skipAllocation = false)
			{
				this->manageData = !skipAllocation;
				this->levelId = levelId;
				this->iterationType = iterationType;

				if (!skipAllocation) this->depth = new ImageType(imgSize, memoryType);
			}

			void UpdateHostFromDevice()
			{ 
				this->depth->UpdateHostFromDevice();
			}

			void UpdateDeviceFromHost()
			{ 
				this->depth->UpdateHostFromDevice();
			}

			~ITMTemplatedHierarchyLevel(void)
			{
				if (manageData) delete depth;
			}

			// Suppress the default copy constructor and assignment operator
			ITMTemplatedHierarchyLevel(const ITMTemplatedHierarchyLevel&);
			ITMTemplatedHierarchyLevel& operator=(const ITMTemplatedHierarchyLevel&);
		};
	}
}
