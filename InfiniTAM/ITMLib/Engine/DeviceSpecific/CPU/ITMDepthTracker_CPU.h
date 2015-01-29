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
			int ComputeGandH(ITMSceneHierarchyLevel *sceneHierarchyLevel, ITMTemplatedHierarchyLevel<ITMFloatImage> *viewHierarchyLevel,
				Matrix4f approxInvPose, Matrix4f imagePose, TrackerIterationType iterationType);

		public:
			ITMDepthTracker_CPU(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
				const ITMLowLevelEngine *lowLevelEngine);
			~ITMDepthTracker_CPU(void);
		};
	}
}