// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"
#include "../Utils/ITMLibSettings.h"

#include "../Objects/ITMTrackingState.h"
#include "../Objects/ITMRenderState.h"

#include "../Engine/ITMVisualisationEngine.h"
#include "../Engine/ITMLowLevelEngine.h"

#include "../Engine/DeviceSpecific/CPU/ITMColorTracker_CPU.h"
#include "../Engine/DeviceSpecific/CPU/ITMDepthTracker_CPU.h"
#include "../Engine/DeviceSpecific/CPU/ITMRenTracker_CPU.h"

#ifndef COMPILE_WITHOUT_CUDA
#include "../Engine/DeviceSpecific/CUDA/ITMColorTracker_CUDA.h"
#include "../Engine/DeviceSpecific/CUDA/ITMDepthTracker_CUDA.h"
#include "../Engine/DeviceSpecific/CUDA/ITMRenTracker_CUDA.h"
#endif

#include "ITMCompositeTracker.h"
#include "ITMIMUTracker.h"
#include "ITMTracker.h"

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
			ITMLibSettings::TrackerType trackerType;

			ITMRenderState *renderState_live;

		public:
			void Track(ITMTrackingState *trackingState, const ITMView *view);
			void Prepare(ITMTrackingState *trackingState, const ITMView *view);

			template <typename TVoxel, typename TIndex>
			ITMTrackingController(const ITMLibSettings *settings, const IITMVisualisationEngine *visualisationEngine,
				const ITMLowLevelEngine *lowLevelEngine, ITMScene<TVoxel, TIndex> *scene, ITMRenderState *renderState_live)
			{
				this->settings = settings;

				this->renderState_live = renderState_live;
				
				this->visualisationEngine = visualisationEngine;
				this->lowLevelEngine = lowLevelEngine;

				trackedImageSize = renderState_live->raycastImage->noDims;

				memoryType = MEMORYDEVICE_CPU;
				trackerType = settings->trackerType;

				switch (settings->deviceType)
				{
				case ITMLibSettings::DEVICE_CPU:
					switch (trackerType)
					{
					case ITMLibSettings::TRACKER_ICP:
					{
						tracker = new ITMDepthTracker_CPU(trackedImageSize, settings->trackingRegime, settings->noHierarchyLevels, settings->noICPRunTillLevel,
							settings->depthTrackerICPThreshold, lowLevelEngine);
						break;
					}
					case ITMLibSettings::TRACKER_IMU:
					{
						ITMCompositeTracker *compositeTracker = new ITMCompositeTracker(2);
						compositeTracker->SetTracker(new ITMIMUTracker(), 0);
						compositeTracker->SetTracker(new ITMDepthTracker_CPU(trackedImageSize, settings->trackingRegime, settings->noHierarchyLevels,
							settings->noICPRunTillLevel, settings->depthTrackerICPThreshold, lowLevelEngine), 1);

						tracker = compositeTracker;
						break;
					}
					case ITMLibSettings::TRACKER_COLOR:
					{
						tracker = new ITMColorTracker_CPU(trackedImageSize, settings->trackingRegime, settings->noHierarchyLevels, lowLevelEngine);
						break;
					}
					case ITMLibSettings::TRACKER_REN:
					{
						ITMCompositeTracker *compositeTracker = new ITMCompositeTracker(2);
						compositeTracker->SetTracker(new ITMDepthTracker_CPU(trackedImageSize, settings->trackingRegime, settings->noHierarchyLevels,
							settings->noICPRunTillLevel, settings->depthTrackerICPThreshold, lowLevelEngine), 0);
						compositeTracker->SetTracker(new ITMRenTracker_CPU<TVoxel, TIndex>(trackedImageSize, lowLevelEngine, scene), 1);

						tracker = compositeTracker;
						break;
					}
					default: break;
					}
					break;
				case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
					switch (trackerType)
					{
					case ITMLibSettings::TRACKER_ICP:
					{
						tracker = new ITMDepthTracker_CUDA(trackedImageSize, settings->trackingRegime, settings->noHierarchyLevels, settings->noICPRunTillLevel,
							settings->depthTrackerICPThreshold, lowLevelEngine);
						break;
					}
					case ITMLibSettings::TRACKER_IMU:
					{
						ITMCompositeTracker *compositeTracker = new ITMCompositeTracker(2);
						compositeTracker->SetTracker(new ITMIMUTracker(), 0);
						compositeTracker->SetTracker(new ITMDepthTracker_CUDA(trackedImageSize, settings->trackingRegime, settings->noHierarchyLevels,
							settings->noICPRunTillLevel, settings->depthTrackerICPThreshold, lowLevelEngine), 1);

						tracker = compositeTracker;
						break;
					}
					case ITMLibSettings::TRACKER_COLOR:
					{
						tracker = new ITMColorTracker_CUDA(trackedImageSize, settings->trackingRegime, settings->noHierarchyLevels, lowLevelEngine);
						break;
					}
					case ITMLibSettings::TRACKER_REN:
					{
						ITMCompositeTracker *compositeTracker = new ITMCompositeTracker(2);
						compositeTracker->SetTracker(new ITMDepthTracker_CUDA(trackedImageSize, settings->trackingRegime, settings->noHierarchyLevels,
							settings->noICPRunTillLevel, settings->depthTrackerICPThreshold, lowLevelEngine), 0);
						compositeTracker->SetTracker(new ITMRenTracker_CUDA<TVoxel, TIndex>(trackedImageSize, lowLevelEngine, scene), 1);
						tracker = compositeTracker;
						break;
					}
					default: break;
					}
					memoryType = MEMORYDEVICE_CUDA;
#endif
					break;
				case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL

#endif
					break;
				default: break;
				}
			}

			~ITMTrackingController();

			ITMTrackingState *BuildTrackingState()
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
