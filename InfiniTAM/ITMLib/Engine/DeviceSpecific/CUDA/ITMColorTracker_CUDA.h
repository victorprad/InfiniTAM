// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMColorTracker.h"

namespace ITMLib
{
	namespace Engine
	{
		class ITMColorTracker_CUDA : public ITMColorTracker
		{
		private:
			Vector2f *f_device; float *g_device, *h_device;
			Vector2f *f_host; float *g_host, *h_host;

		public:
			void F_oneLevel(float *f, ITMPose *pose);
			void G_oneLevel(float *gradient, float *hessian, ITMPose *pose) const;

			ITMColorTracker_CUDA(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
				const ITMLowLevelEngine *lowLevelEngine);
			~ITMColorTracker_CUDA(void);
		};
	}
}
