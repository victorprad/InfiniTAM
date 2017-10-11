// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMColorTracker.h"

namespace ITMLib
{
	class ITMColorTracker_CUDA : public ITMColorTracker
	{
	private:
		Vector2f *f_device; float *g_device, *h_device;
		Vector2f *f_host; float *g_host, *h_host;

	public:
		int F_oneLevel(float *f, ORUtils::SE3Pose *pose);
		void G_oneLevel(float *gradient, float *hessian, ORUtils::SE3Pose *pose) const;

		ITMColorTracker_CUDA(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
			const ITMLowLevelEngine *lowLevelEngine);
		~ITMColorTracker_CUDA(void);
	};
}
