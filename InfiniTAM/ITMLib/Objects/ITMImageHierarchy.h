// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

namespace ITMLib
{
	namespace Objects
	{
		template <class T> class ITMImageHierarchy
		{
		public:
			int noLevels;
			T **levels;

			ITMImageHierarchy(Vector2i imgSize, int noHierarchyLevels, int noRotationOnlyLevels, bool useGPU)
			{
				this->noLevels = noHierarchyLevels;

				levels = new T*[noHierarchyLevels];

				int currentRotationOnlyLevel = 0;
				for (int i = noHierarchyLevels - 1; i >= 0; i--)
				{
					bool currentLevelRotationOnly = (currentRotationOnlyLevel < noRotationOnlyLevels) ? true : false;
					levels[i] = new T(imgSize, i, currentLevelRotationOnly, useGPU);
					currentRotationOnlyLevel++;
				}
			}

			void UpdateHostFromDevice()
			{ for (int i = 0; i < noLevels; i++) this->levels[i]->UpdateHostFromDevice(); }

			void UpdateDeviceFromHost()
			{ for (int i = 0; i < noLevels; i++) this->levels[i]->UpdateDeviceFromHost(); }

			~ITMImageHierarchy(void)
			{
				for (int i = 0; i < noLevels; i++) delete levels[i];
				delete [] levels;
			}

			// Suppress the default copy constructor and assignment operator
			ITMImageHierarchy(const ITMImageHierarchy&);
			ITMImageHierarchy& operator=(const ITMImageHierarchy&);
		};
	}
}
