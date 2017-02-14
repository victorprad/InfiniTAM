// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Objects/Tracking/ITMTrackingState.h"
#include "../../Objects/Views/ITMView.h"

namespace ITMLib
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

		virtual bool requiresColourRendering(void) const = 0;
		virtual bool requiresDepthReliability(void) const = 0;

		virtual ~ITMTracker(void) {}
	};
}
