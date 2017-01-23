// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMTracker.h"

namespace ITMLib
{
	class ITMCompositeTracker : public ITMTracker
	{
	private:
		ITMTracker **trackers; int noTrackers;
	public:

		void SetTracker(ITMTracker *tracker, int trackerId)
		{
			if (trackers[trackerId] != NULL) delete trackers[trackerId];
			trackers[trackerId] = tracker;
		}

		ITMCompositeTracker(int noTrackers)
		{
			trackers = new ITMTracker*[noTrackers];
			for (int i = 0; i < noTrackers; i++) trackers[i] = NULL;

			this->noTrackers = noTrackers;
		}

		~ITMCompositeTracker(void)
		{
			for (int i = 0; i < noTrackers; i++)
				if (trackers[i] != NULL) delete trackers[i];

			delete [] trackers;
		}

		void TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
		{
			for (int i = 0; i < noTrackers; i++) trackers[i]->TrackCamera(trackingState, view);
		}

		void UpdateInitialPose(ITMTrackingState *trackingState)
		{
			for (int i = 0; i < noTrackers; i++) trackers[i]->UpdateInitialPose(trackingState);
		}

		bool requiresColourRendering(void) const
		{
			for (int i = 0; i < noTrackers; i++) if (trackers[i]->requiresColourRendering()) return true;
			return false;
		}
		bool requiresDepthReliability(void) const
		{
			for (int i = 0; i < noTrackers; i++) if (trackers[i]->requiresDepthReliability()) return true;
			return false;
		}

		bool requiresPointCloudRendering() const
		{
			for (int i = 0; i < noTrackers; i++) if (trackers[i]->requiresPointCloudRendering()) return true;
			return false;
		}

		// Suppress the default copy constructor and assignment operator
		ITMCompositeTracker(const ITMCompositeTracker&);
		ITMCompositeTracker& operator=(const ITMCompositeTracker&);
	};
}

