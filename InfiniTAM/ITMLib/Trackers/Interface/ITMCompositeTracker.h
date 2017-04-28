// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <vector>

#include "ITMTracker.h"

namespace ITMLib
{
	class ITMCompositeTracker : public ITMTracker
	{
		//#################### ENUMERATIONS ####################
	public:
		enum Policy
		{
			POLICY_REFINE,
			POLICY_SEQUENTIAL,
			POLICY_STOP_ON_FIRST_SUCCESS
		};

		//#################### PRIVATE VARIABLES ####################
	private:
		std::vector<ITMTracker*> trackers;
		Policy trackingPolicy;

		//#################### CONSTRUCTORS ####################
	public:
		explicit ITMCompositeTracker(Policy trackingPolicy = POLICY_REFINE)
		: trackingPolicy(trackingPolicy)
		{}

		//#################### DESTRUCTOR ####################
	public:
		~ITMCompositeTracker()
		{
			for (size_t i = 0, size = trackers.size(); i < size; ++i)
			{
				delete trackers[i];
			}
		}

		//#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
	private:
		// Deliberately private and unimplemented
		ITMCompositeTracker(const ITMCompositeTracker&);
		ITMCompositeTracker& operator=(const ITMCompositeTracker&);

		//#################### PUBLIC MEMBER FUNCTIONS ####################
	public:
		void AddTracker(ITMTracker *tracker)
		{
			trackers.push_back(tracker);
		}

		bool CanKeepTracking() const
		{
			if (trackingPolicy == POLICY_REFINE)
			{
				// All of the trackers must still be able to track when using refine.
				for (size_t i = 0, size = trackers.size(); i < size; ++i)
				{
					if (!trackers[i]->CanKeepTracking()) return false;
				}
				return true;
			}
			else
			{
				// Only one of the trackers must still be able to track when using other policies.
				for (size_t i = 0, size = trackers.size(); i < size; ++i)
				{
					if (trackers[i]->CanKeepTracking()) return true;
				}
				return false;
			}
		}

		void TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
		{
			trackingState->trackerResult = ITMTrackingState::TRACKING_FAILED;

			for (size_t i = 0, size = trackers.size(); i < size; ++i)
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
			for (size_t i = 0, size = trackers.size(); i < size; ++i)
			{
				trackers[i]->UpdateInitialPose(trackingState);
			}
		}

		bool requiresColourRendering() const
		{
			for (size_t i = 0, size = trackers.size(); i < size; ++i)
			{
				if (trackers[i]->requiresColourRendering()) return true;
			}
			return false;
		}

		bool requiresDepthReliability() const
		{
			for (size_t i = 0, size = trackers.size(); i < size; ++i)
			{
				if (trackers[i]->requiresDepthReliability()) return true;
			}
			return false;
		}

		bool requiresPointCloudRendering() const
		{
			for (size_t i = 0, size = trackers.size(); i < size; ++i)
			{
				if (trackers[i]->requiresPointCloudRendering()) return true;
			}
			return false;
		}
	};
}
