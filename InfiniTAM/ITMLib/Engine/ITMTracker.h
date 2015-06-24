// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMTrackingState.h"
#include "../Objects/ITMView.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		/** \brief
		    Basic interface to any sort of trackers that will align an
		    incoming view with an existing scene.
		*/
		class ITMTracker
		{
		public:
			/** Localize a View in the given scene. The result is
			    currently stored as an attribute in trackingState.
			*/
			virtual void TrackCamera(ITMTrackingState *trackingState, const ITMView *view) = 0;

			/** Updates the initial pose of the depth camera in the scene.
			    This can be used to make the scene up vector correspond
			    to the real world's up direction.
			*/
			virtual void UpdateInitialPose(ITMTrackingState *trackingState) {}

			virtual ~ITMTracker(void) {}
		};
	}
}
