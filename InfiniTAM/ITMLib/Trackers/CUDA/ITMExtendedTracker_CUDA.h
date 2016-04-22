// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMExtendedTracker.h"

namespace ITMLib
{
	class ITMExtendedTracker_CUDA : public ITMExtendedTracker
	{
	public:
		struct AccuCell;

	private:
		AccuCell *accu_host;
		AccuCell *accu_device;

	protected:
		int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);

	public:
		ITMExtendedTracker_CUDA(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
			float terminationThreshold, float failureDetectorThreshold, float viewFrustum_min, float viewFrustum_max, 
			float tukeyCutOff, float framesToSkip, float framesToWeight, const ITMLowLevelEngine *lowLevelEngine);
		~ITMExtendedTracker_CUDA(void);
	};
}
