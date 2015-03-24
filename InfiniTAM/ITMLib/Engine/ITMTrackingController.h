// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMTrackingState.h"
#include "../Objects/ITMRenderState.h"

#include "../Engine/ITMVisualisationEngine.h"
#include "../Engine/ITMLowLevelEngine.h"

#include "ITMTrackerFactory.h"

namespace ITMLib
{
	namespace Engine
	{
		/** \brief
		*/
		class ITMTrackingController
		{
		private:
			const ITMLibSettings *settings;
			const IITMVisualisationEngine *visualisationEngine;
			const ITMLowLevelEngine *lowLevelEngine;

			ITMTracker *tracker;

			Vector2i trackedImageSize;

			MemoryDeviceType memoryType;

			ITMRenderState *renderState_live;

			bool IsFarFromPrevious(const ITMTrackingState *trackingState) const;

		public:
			void Track(ITMTrackingState *trackingState, const ITMView *view);
			void Prepare(ITMTrackingState *trackingState, const ITMView *view);

			ITMTrackingController(ITMTracker *tracker, const IITMVisualisationEngine *visualisationEngine, const ITMLowLevelEngine *lowLevelEngine,
				ITMRenderState *renderState_live, const ITMLibSettings *settings)
			{
				this->tracker = tracker;
				this->settings = settings;
				this->renderState_live = renderState_live;
				this->visualisationEngine = visualisationEngine;
				this->lowLevelEngine = lowLevelEngine;

				trackedImageSize = renderState_live->raycastImage->noDims;
				memoryType = settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
			}

			ITMTrackingState *BuildTrackingState() const
			{
				return new ITMTrackingState(trackedImageSize, memoryType);
			}

			static Vector2i GetTrackedImageSize(const ITMLibSettings *settings, const Vector2i& imgSize_rgb, const Vector2i& imgSize_d)
			{
				return settings->trackerType == ITMLibSettings::TRACKER_COLOR ? imgSize_rgb : imgSize_d;
			}

			// Suppress the default copy constructor and assignment operator
			ITMTrackingController(const ITMTrackingController&);
			ITMTrackingController& operator=(const ITMTrackingController&);
		};
	}
}
