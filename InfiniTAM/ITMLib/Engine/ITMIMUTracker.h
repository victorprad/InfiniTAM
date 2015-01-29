// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMIMUMeasurement.h"

#include "../Engine/ITMTracker.h"
#include "../Engine/ITMLowLevelEngine.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		class ITMIMUTracker : public ITMTracker
		{
		private:
			ITMTrackingState *trackingState; const ITMView *view;
			ITMPose *imuPose_imucoords, *imuPose_cameracoords;
			Matrix3f oldR_visual;
			bool hasAtLeastTwoFrames;

		public:
			void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

			ITMIMUTracker();
			virtual ~ITMIMUTracker(void);
		};
	}
}
