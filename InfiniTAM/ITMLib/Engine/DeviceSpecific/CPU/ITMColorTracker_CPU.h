// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMColorTracker.h"

namespace ITMLib
{
	namespace Engine
	{
		class ITMColorTracker_CPU : public ITMColorTracker
		{
		public:
			void F_oneLevel(float *f, ITMPose *pose);
			void G_oneLevel(float *gradient, float *hessian, ITMPose *pose) const;

			ITMColorTracker_CPU(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
				const ITMLowLevelEngine *lowLevelEngine);
			~ITMColorTracker_CPU(void);
		};
	}
}
