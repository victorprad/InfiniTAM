// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "TrackerIterationType.h"
#include "../../Utils/ITMMath.h"
#include "../../../ORUtils/MemoryBlock.h"

namespace ITMLib
{
	template <class T0, class T1> class ITMTwoImageHierarchy
	{
	public:
		int noLevels;
		T0 **levels_t0;
		T1 **levels_t1;

		ITMTwoImageHierarchy(Vector2i imgSize_t1, Vector2i imgSize_t2, TrackerIterationType *trackingRegime, int noHierarchyLevels,
			MemoryDeviceType memoryType, bool skipAllocationForLevel0 = false)
		{
			this->noLevels = noHierarchyLevels;

			levels_t0 = new T0*[noHierarchyLevels];
			levels_t1 = new T1*[noHierarchyLevels];

			for (int i = noHierarchyLevels - 1; i >= 0; i--)
			{
				levels_t0[i] = new T0(imgSize_t1, i, trackingRegime[i], memoryType, (i == 0) && skipAllocationForLevel0);
				levels_t1[i] = new T1(imgSize_t1, i, trackingRegime[i], memoryType, (i == 0) && skipAllocationForLevel0);
			}
		}

		ITMTwoImageHierarchy(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
			MemoryDeviceType memoryType, int levelToAllocate = 0, bool skipAllocationForLevel0 = false)
		{
			this->noLevels = noHierarchyLevels;

			if (levelToAllocate == 0)
			{
				levels_t0 = new T0*[noHierarchyLevels];
				levels_t1 = NULL;
			}
			else
			{
				levels_t0 = NULL;
				levels_t1 = new T1*[noHierarchyLevels];
			}

			if (levels_t0 != NULL) for (int i = noHierarchyLevels - 1; i >= 0; i--)
				levels_t0[i] = new T0(imgSize, i, trackingRegime[i], memoryType, (i == 0) && skipAllocationForLevel0);

			if (levels_t1 != NULL) for (int i = noHierarchyLevels - 1; i >= 0; i--)
				levels_t1[i] = new T1(imgSize, i, trackingRegime[i], memoryType, (i == 0) && skipAllocationForLevel0);
		}

		void UpdateHostFromDevice()
		{ 
			if (levels_t0 != NULL) for (int i = 0; i < noLevels; i++) this->levels_t0[i]->UpdateHostFromDevice();
			if (levels_t1 != NULL) for (int i = 0; i < noLevels; i++) this->levels_t1[i]->UpdateHostFromDevice();
				
		}

		void UpdateDeviceFromHost()
		{ 
			if (levels_t0 != NULL) for (int i = 0; i < noLevels; i++) this->levels_t0[i]->UpdateDeviceFromHost();
			if (levels_t1 != NULL) for (int i = 0; i < noLevels; i++) this->levels_t1[i]->UpdateDeviceFromHost();
		}

		~ITMTwoImageHierarchy(void)
		{
			if (levels_t0 != NULL)
			{
				for (int i = 0; i < noLevels; i++) delete levels_t0[i];
				delete[] levels_t0;
			}
			if (levels_t1 != NULL)
			{
				for (int i = 0; i < noLevels; i++) delete levels_t1[i];
				delete[] levels_t1;
			}
		}

		// Suppress the default copy constructor and assignment operator
		ITMTwoImageHierarchy(const ITMTwoImageHierarchy&);
		ITMTwoImageHierarchy& operator=(const ITMTwoImageHierarchy&);
	};
}
