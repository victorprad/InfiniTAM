// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMTrackingState.h"
#include "../Objects/ITMRenderState.h"

#include "../Engine/ITMVisualisationEngine.h"

#include "ITMTrackerFactory.h"

namespace ITMLib
{
	/** \brief
	*/
	class ITMTrackingController
	{
	private:
		const ITMLibSettings *settings;
		const IITMVisualisationEngine *visualisationEngine;

		ITMTracker *tracker;

	public:
		void Track(ITMTrackingState *trackingState, const ITMView *view);
		void Prepare(ITMTrackingState *trackingState, const ITMSceneBase *scene, const ITMView *view, ITMRenderState *renderState);

		ITMTrackingController(ITMTracker *tracker, const IITMVisualisationEngine *visualisationEngine, const ITMLibSettings *settings)
		{
			this->tracker = tracker;
			this->settings = settings;
			this->visualisationEngine = visualisationEngine;
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
