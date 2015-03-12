// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

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

			/** Sets the initial pose of the depth camera in the scene.
					This can be used to make the up vector in our scene correspond
					to the real world's up direction.
			*/
			virtual void SetInitialPose(ITMTrackingState *trackingState)
			{
				// By default (i.e. in the absence of any hardware support), we don't know the pose of
				// our camera in the real world, so using the identity matrix is the best we can do.
				Matrix4f M;
				M.setIdentity();
				trackingState->pose_d->SetM(M);
			}

			virtual ~ITMTracker(void) {}
		};
	}
}
