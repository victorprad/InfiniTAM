// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"
#include "../Engine/ITMTracker.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
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

			// Suppress the default copy constructor and assignment operator
			ITMCompositeTracker(const ITMCompositeTracker&);
			ITMCompositeTracker& operator=(const ITMCompositeTracker&);
		};
	}
}
