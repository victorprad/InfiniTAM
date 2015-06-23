// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMDepthTracker.h"

namespace ITMLib
{
	namespace Engine
	{
		class ITMDepthTracker_CPU : public ITMDepthTracker
		{
		protected:
			int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);

		public:
			ITMDepthTracker_CPU(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
				float terminationThreshold, const ITMLowLevelEngine *lowLevelEngine);
			~ITMDepthTracker_CPU(void);
		};
	}
}