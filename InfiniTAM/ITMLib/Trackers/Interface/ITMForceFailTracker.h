// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMTracker.h"

namespace ITMLib
{
	/**
	 * \brief An instance of this class can be used to force tracking failure,
	 *        e.g. when testing a relocaliser.
	 */
	class ITMForceFailTracker : public ITMTracker
	{
		//#################### PUBLIC MEMBER FUNCTIONS ####################
	public:
		/** Override */
		virtual void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

		/** Override */
		virtual bool requiresColourRendering() const;

		/** Override */
		virtual bool requiresDepthReliability() const;

		/** Override */
		virtual bool requiresPointCloudRendering() const;
	};
}
