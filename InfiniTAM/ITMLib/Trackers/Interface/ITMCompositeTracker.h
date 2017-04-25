// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMTracker.h"

namespace ITMLib
{
	class ITMCompositeTracker : public ITMTracker
	{
	public:
		enum Policy
		{
			POLICY_REFINE,
			POLICY_SEQUENTIAL,
			POLICY_STOP_ON_FIRST_SUCCESS
		};

	private:
		ITMTracker **trackers;
		int noTrackers;
		Policy trackingPolicy;

	public:
		void SetTracker(ITMTracker *tracker, int trackerId)
		{
			delete trackers[trackerId];
			trackers[trackerId] = tracker;
		}

		explicit ITMCompositeTracker(int noTrackers, Policy trackingPolicy = POLICY_REFINE)
			: noTrackers(noTrackers)
			, trackingPolicy(trackingPolicy)
		{
			trackers = new ITMTracker*[noTrackers];
			for (int i = 0; i < noTrackers; i++) trackers[i] = NULL;
		}

		~ITMCompositeTracker()
		{
			for (int i = 0; i < noTrackers; i++)
				delete trackers[i];

			delete [] trackers;
		}

		bool CanKeepTracking() const
		{
			for (int i = 0; i < noTrackers; i++)
			{
				if (trackers[i]->CanKeepTracking()) return true;
			}
			return false;
		}

		void TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
		{
			for (int i = 0; i < noTrackers; i++)
			{
				if (!trackers[i]->CanKeepTracking()) continue;

				trackers[i]->TrackCamera(trackingState, view);

				if (trackingPolicy == POLICY_SEQUENTIAL ||
				    (trackingPolicy == POLICY_STOP_ON_FIRST_SUCCESS && trackingState->trackerResult == ITMTrackingState::TRACKING_GOOD))
				{
					break;
				}
			}
		}

		void UpdateInitialPose(ITMTrackingState *trackingState)
		{
			for (int i = 0; i < noTrackers; i++) trackers[i]->UpdateInitialPose(trackingState);
		}

		bool requiresColourRendering() const
		{
			for (int i = 0; i < noTrackers; i++) if (trackers[i]->requiresColourRendering()) return true;
			return false;
		}

		bool requiresDepthReliability() const
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
