// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMDepthTracker.h"

namespace ITMLib
{
	namespace Engine
	{
		class ITMDepthTracker_CPU : public ITMDepthTracker
		{
		protected:
			int ComputeGandH(float &f, float *ATb_host, float *ATA_host, Matrix4f approxInvPose);

		public:
			ITMDepthTracker_CPU(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
				const ITMLowLevelEngine *lowLevelEngine);
			~ITMDepthTracker_CPU(void);
		};
	}
}