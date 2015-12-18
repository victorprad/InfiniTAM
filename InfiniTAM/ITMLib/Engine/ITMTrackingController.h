// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "../Trackers/Base/ITMTrackingState.h"
#include "../Objects/ITMRenderState.h"

#include "../Visualisation/Interface/ITMVisualisationEngine.h"

#include "../Trackers/ITMTrackerFactory.h"

namespace ITMLib
{
	/** \brief
	*/
	class ITMTrackingController
	{
	private:
		const ITMLibSettings *settings;
		ITMTracker *tracker;

	public:
		void Track(ITMTrackingState *trackingState, const ITMView *view);

		template <typename TVoxel, typename TIndex>
		void Prepare(ITMTrackingState *trackingState, const ITMScene<TVoxel,TIndex> *scene, const ITMView *view,
			const ITMVisualisationEngine<TVoxel,TIndex> *visualisationEngine, ITMRenderState *renderState)
		{
			//render for tracking
			bool requiresColourRendering = tracker->requiresColourRendering();
			bool requiresFullRendering = trackingState->TrackerFarFromPointCloud() || !settings->useApproximateRaycast;

			if (requiresColourRendering)
			{
				ITMPose pose_rgb(view->calib->trafo_rgb_to_depth.calib_inv * trackingState->pose_d->GetM());
				visualisationEngine->CreateExpectedDepths(scene, &pose_rgb, &(view->calib->intrinsics_rgb), renderState);
				visualisationEngine->CreatePointCloud(scene, view, trackingState, renderState, settings->skipPoints);
				trackingState->age_pointCloud = 0;
			}
			else
			{
				visualisationEngine->CreateExpectedDepths(scene, trackingState->pose_d, &(view->calib->intrinsics_d), renderState);

				if (requiresFullRendering)
				{
					visualisationEngine->CreateICPMaps(scene, view, trackingState, renderState);
					trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
					if (trackingState->age_pointCloud==-1) trackingState->age_pointCloud=-2;
					else trackingState->age_pointCloud = 0;
				}
				else
				{
					visualisationEngine->ForwardRender(scene, view, trackingState, renderState);
					trackingState->age_pointCloud++;
				}
			}
		}

		ITMTrackingController(ITMTracker *tracker, const ITMLibSettings *settings)
		{
			this->tracker = tracker;
			this->settings = settings;
		}

		const Vector2i& GetTrackedImageSize(const Vector2i& imgSize_rgb, const Vector2i& imgSize_d) const
		{
			return tracker->requiresColourRendering() ? imgSize_rgb : imgSize_d;
		}

		// Suppress the default copy constructor and assignment operator
		ITMTrackingController(const ITMTrackingController&);
		ITMTrackingController& operator=(const ITMTrackingController&);
	};
}
