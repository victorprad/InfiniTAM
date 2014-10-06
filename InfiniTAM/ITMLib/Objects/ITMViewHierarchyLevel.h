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

			ITMViewHierarchyLevel(Vector2i imgSize, int levelId, bool rotationOnly, bool useGPU)
			{
				this->levelId = levelId;
				this->rotationOnly = rotationOnly;

				this->rgb = new ITMUChar4Image(imgSize, useGPU);
				this->depth = new ITMFloatImage(imgSize, useGPU);
				this->gradientX_rgb = new ITMShort4Image(imgSize, useGPU);
				this->gradientY_rgb = new ITMShort4Image(imgSize, useGPU);
			}

			void UpdateHostFromDevice()
			{ 
				this->rgb->UpdateHostFromDevice();
				this->depth->UpdateHostFromDevice();
				this->gradientX_rgb->UpdateHostFromDevice();
			}

			void UpdateDeviceFromHost()
			{ 
				this->rgb->UpdateDeviceFromHost();
				this->depth->UpdateHostFromDevice();
				this->gradientX_rgb->UpdateDeviceFromHost();
			}

			~ITMViewHierarchyLevel(void)
			{
				delete rgb;
				delete depth;
				delete gradientX_rgb; delete gradientY_rgb;
			}

			// Suppress the default copy constructor and assignment operator
			ITMViewHierarchyLevel(const ITMViewHierarchyLevel&);
			ITMViewHierarchyLevel& operator=(const ITMViewHierarchyLevel&);
		};
	}
}
