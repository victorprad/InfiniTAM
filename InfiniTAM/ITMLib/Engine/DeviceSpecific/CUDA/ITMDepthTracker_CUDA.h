// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMDepthTracker.h"

namespace ITMLib
{
	namespace Engine
	{
		class ITMDepthTracker_CUDA : public ITMDepthTracker
		{
		public:
			struct AccuCell;

		private:
			AccuCell *accu_host;
			AccuCell *accu_device;

		protected:
			int ComputeGandH(float &f, float *ATb_host, float *ATA_host, Matrix4f approxInvPose);

		public:
			ITMDepthTracker_CUDA(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
				const ITMLowLevelEngine *lowLevelEngine);
			~ITMDepthTracker_CUDA(void);
		};
	}
}
