// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Objects/ITMImage.h"

namespace ITMLib
{
	namespace Objects
	{
		template <class ImageType>
		class ITMTemplatedHierarchyLevel
		{
		public:
			int levelId;

			bool rotationOnly;

			ImageType *depth;
			Vector4f intrinsics;

			ITMTemplatedHierarchyLevel(Vector2i imgSize, int levelId, bool rotationOnly, bool useGPU)
			{
				this->levelId = levelId;
				this->rotationOnly = rotationOnly;

				this->depth = new ImageType(imgSize, useGPU);
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
				delete depth;
			}

			// Suppress the default copy constructor and assignment operator
			ITMTemplatedHierarchyLevel(const ITMTemplatedHierarchyLevel&);
			ITMTemplatedHierarchyLevel& operator=(const ITMTemplatedHierarchyLevel&);
		};
	}
}
