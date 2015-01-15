// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Objects/ITMImage.h"

namespace ITMLib
{
	namespace Objects
	{
		class ITMViewHierarchyLevel
		{
		public:
			int levelId;

			bool rotationOnly;

			ITMUChar4Image *rgb; ITMFloatImage *depth;
			ITMShort4Image *gradientX_rgb, *gradientY_rgb;
			Vector4f intrinsics;

			bool manageData;

			ITMViewHierarchyLevel(Vector2i imgSize, int levelId, bool rotationOnly, bool useGPU, bool skipAllocation)
			{
				this->manageData = !skipAllocation;
				this->levelId = levelId;
				this->rotationOnly = rotationOnly;

				if (!skipAllocation) {
					this->rgb = new ITMUChar4Image(imgSize, true, useGPU);
					this->depth = new ITMFloatImage(imgSize, true, useGPU);
					this->gradientX_rgb = new ITMShort4Image(imgSize, true, useGPU);
					this->gradientY_rgb = new ITMShort4Image(imgSize, true, useGPU);
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
