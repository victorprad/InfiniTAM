// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMExtendedTracker.h"

namespace ITMLib
{
	class ITMExtendedTracker_CPU : public ITMExtendedTracker
	{
	protected:
		int ComputeGandH_Depth(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);
		int ComputeGandH_RGB(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);

	public:
		ITMExtendedTracker_CPU(Vector2i imgSize_d, Vector2i imgSize_rgb, bool useDepth, bool useColour,
			TrackerIterationType *trackingRegime, int noHierarchyLevels,
			float terminationThreshold, float failureDetectorThreshold, float viewFrustum_min, float viewFrustum_max, 
			int tukeyCutOff, int framesToSkip, int framesToWeight, const ITMLowLevelEngine *lowLevelEngine);
		~ITMExtendedTracker_CPU(void);
	};
}
