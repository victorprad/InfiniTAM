// Copyright 2016 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMTracker.h"

namespace ITMLib
{
	/**
	 * \brief Tracker that reads precomputed poses from text files.
	 */
	class ITMFileBasedTracker : public ITMTracker
	{
	private:
		std::string poseMask;
		size_t frameCount;

	public:
		void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

		bool requiresColourRendering() const { return false; }
		bool requiresDepthReliability() const { return false; }

		explicit ITMFileBasedTracker(const std::string &poseMask);
		virtual ~ITMFileBasedTracker();
	};
}
