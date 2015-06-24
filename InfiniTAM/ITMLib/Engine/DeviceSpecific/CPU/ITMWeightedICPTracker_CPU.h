// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMWeightedICPTracker.h"

namespace ITMLib
{
	namespace Engine
	{
		class ITMWeightedICPTracker_CPU : public ITMWeightedICPTracker
		{
		protected:
			int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);

		public:
			ITMWeightedICPTracker_CPU(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
				float terminationThreshold, const ITMLowLevelEngine *lowLevelEngine);
			~ITMWeightedICPTracker_CPU(void);
		};
	}
}