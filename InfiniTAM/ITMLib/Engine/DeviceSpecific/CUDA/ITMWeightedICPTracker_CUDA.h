// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMWeightedICPTracker.h"

namespace ITMLib
{
	namespace Engine
	{
		class ITMWeightedICPTracker_CUDA : public ITMWeightedICPTracker
		{
		public:
			struct AccuCell;

		private:
			AccuCell *accu_host;
			AccuCell *accu_device;

		protected:
			int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);

		public:
			ITMWeightedICPTracker_CUDA(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
				float terminationThreshold, const ITMLowLevelEngine *lowLevelEngine);
			~ITMWeightedICPTracker_CUDA(void);
		};
	}
}
