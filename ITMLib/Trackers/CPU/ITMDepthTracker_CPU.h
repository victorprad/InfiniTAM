// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMDepthTracker.h"

namespace ITMLib
{
	class ITMDepthTracker_CPU : public ITMDepthTracker
	{
	protected:
		int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);

	public:
		ITMDepthTracker_CPU(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
			float terminationThreshold, float failureDetectorThreshold, const ITMLowLevelEngine *lowLevelEngine);
		~ITMDepthTracker_CPU(void);
	};
}
