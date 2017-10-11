// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "TrackerIterationType.h"
#include "../../Utils/ITMMath.h"
#include "../../../ORUtils/MemoryBlock.h"

namespace ITMLib
{
	template <class T> class ITMImageHierarchy
	{
	private:
		int noLevels;
		T **levels;

	public:
		ITMImageHierarchy(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, 
			MemoryDeviceType memoryType, bool skipAllocationForLevel0 = false)
		{
			this->noLevels = noHierarchyLevels;

			levels = new T*[noHierarchyLevels];

			for (int i = noHierarchyLevels - 1; i >= 0; i--)
				levels[i] = new T(imgSize, i, trackingRegime[i], memoryType, (i == 0) && skipAllocationForLevel0);
		}

		void UpdateHostFromDevice()
		{ for (int i = 0; i < noLevels; i++) this->levels[i]->UpdateHostFromDevice(); }

		void UpdateDeviceFromHost()
		{ for (int i = 0; i < noLevels; i++) this->levels[i]->UpdateDeviceFromHost(); }

		int GetNoLevels() const { return noLevels; }

		T * GetLevel(int level) const
		{
			return level >= 0 && level < noLevels ? levels[level] : NULL;
		}

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
