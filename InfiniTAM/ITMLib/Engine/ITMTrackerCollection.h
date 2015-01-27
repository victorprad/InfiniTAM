// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "../Engine/ITMTracker.h"
#include "../Engine/ITMLowLevelEngine.h"
#include "../Engine/ITMDenseMapper.h"

#include "../Utils/ITMLibSettings.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		class ITMTrackerCollection : public ITMTracker
		{
		private:
			ITMLowLevelEngine *lowLevelEngine;
			ITMTracker **trackers; int noTrackers;

			ITMLibSettings::TrackerType trackerType;
			Vector2i trackedImageSize;
			MemoryDeviceType memoryType;

			bool skipPoints;
		public:

			template <typename TVoxel, typename TIndex>
			ITMTrackerCollection(const ITMLibSettings *settings, const Vector2i& imgSize_rgb, const Vector2i& imgSize_d, 
				const ITMDenseMapper<TVoxel, TIndex> *denseMapper, ITMLowLevelEngine *lowLevelEngine)
			{
				trackedImageSize = ITMLibSettings::TRACKER_COLOR ? imgSize_rgb : imgSize_d;
				memoryType = MEMORYDEVICE_CPU;

				trackerType = settings->trackerType;

				switch (settings->deviceType)
				{
				case ITMLibSettings::DEVICE_CPU:
				{
					switch (trackerType)
					{
					case ITMLibSettings::TRACKER_ICP:
						noTrackers = 1;
						trackers = new ITMTracker*[noTrackers] {
							new ITMDepthTracker_CPU(imgSize_d, settings->noHierarchyLevels, settings->noRotationOnlyLevels, settings->noICPRunTillLevel,
								settings->depthTrackerICPThreshold, lowLevelEngine)
						};
						break;
					case ITMLibSettings::TRACKER_IMU:
						noTrackers = 2;
						trackers = new ITMTracker*[noTrackers] {
							new ITMIMUTracker(lowLevelEngine),
							new ITMDepthTracker_CPU(imgSize_d, settings->noHierarchyLevels, settings->noRotationOnlyLevels, settings->noICPRunTillLevel,
								settings->depthTrackerICPThreshold, lowLevelEngine)
						};
						break;
					case ITMLibSettings::TRACKER_COLOR:
						noTrackers = 1;
						trackers = new ITMTracker*[noTrackers] {
							new ITMColorTracker_CPU(imgSize_rgb, settings->noHierarchyLevels, settings->noRotationOnlyLevels, lowLevelEngine)
						};
						skipPoints = settings->skipPoints;
						break;
					case ITMLibSettings::TRACKER_REN:
						noTrackers = 2;
						trackers = new ITMTracker*[noTrackers] {
							new ITMDepthTracker_CPU(imgSize_d, settings->noHierarchyLevels, settings->noRotationOnlyLevels, settings->noICPRunTillLevel,
								settings->depthTrackerICPThreshold, lowLevelEngine),
							new ITMRenTracker_CPU<TVoxel, TIndex>(imgSize_d, settings->noICPRunTillLevel, lowLevelEngine, denseMapper->getScene())
						};
						break;
					default: break;
					}
				}
					break;
				case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
					switch (trackerType)
					{
					case ITMLibSettings::TRACKER_ICP:
						noTrackers = 1;
						trackers = new ITMTracker*[noTrackers] {
							new ITMDepthTracker_CUDA(imgSize_d, settings->noHierarchyLevels, settings->noRotationOnlyLevels, settings->noICPRunTillLevel,
								settings->depthTrackerICPThreshold, lowLevelEngine)
						};
						break;
					case ITMLibSettings::TRACKER_IMU:
						noTrackers = 2;
						trackers = new ITMTracker*[noTrackers] {
							new ITMIMUTracker(lowLevelEngine),
							new ITMDepthTracker_CUDA(imgSize_d, settings->noHierarchyLevels, settings->noRotationOnlyLevels, settings->noICPRunTillLevel,
								settings->depthTrackerICPThreshold, lowLevelEngine)
						};
						break;
					case ITMLibSettings::TRACKER_COLOR:
						noTrackers = 1;
						trackers = new ITMTracker*[noTrackers] {
							new ITMColorTracker_CUDA(imgSize_rgb, settings->noHierarchyLevels, settings->noRotationOnlyLevels, lowLevelEngine)
						};
						skipPoints = settings->skipPoints;
						break;
					case ITMLibSettings::TRACKER_REN:
						noTrackers = 2;
						trackers = new ITMTracker*[noTrackers] {
							new ITMDepthTracker_CUDA(imgSize_d, settings->noHierarchyLevels, settings->noRotationOnlyLevels, settings->noICPRunTillLevel,
								settings->depthTrackerICPThreshold, lowLevelEngine),
							new ITMRenTracker_CUDA<TVoxel, TIndex>(imgSize_d, settings->noICPRunTillLevel, lowLevelEngine, denseMapper->getScene())
						};
						break;
					default: break;
					}

					memoryType = MEMORYDEVICE_CUDA;
#endif
					break;
				case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
					switch (trackerType)
					{
					case ITMLibSettings::TRACKER_ICP:
						noTrackers = 1;
						trackers = new ITMTracker*[noTrackers] {
							new ITMDepthTracker_Metal(imgSize_d, settings->noHierarchyLevels, settings->noRotationOnlyLevels, settings->noICPRunTillLevel,
								settings->depthTrackerICPThreshold, lowLevelEngine)
						};
						break;
					case ITMLibSettings::TRACKER_IMU:
						noTrackers = 2;
						trackers = new ITMTracker*[noTrackers] {
							new ITMIMUTracker(lowLevelEngine),
							new ITMDepthTracker_Metal(imgSize_d, settings->noHierarchyLevels, settings->noRotationOnlyLevels, settings->noICPRunTillLevel,
								settings->depthTrackerICPThreshold, lowLevelEngine)
						};
						break;
					case ITMLibSettings::TRACKER_COLOR:
						noTrackers = 1;
						trackers = new ITMTracker*[noTrackers] {
							new ITMColorTracker_CPU(imgSize_rgb, settings->noHierarchyLevels, settings->noRotationOnlyLevels, lowLevelEngine)
						};
						skipPoints = settings->skipPoints;
						break;
					case ITMLibSettings::TRACKER_REN:
						noTrackers = 2;
						trackers = new ITMTracker*[noTrackers] {
							new ITMDepthTracker_CPU(imgSize_d, settings->noHierarchyLevels, settings->noRotationOnlyLevels, settings->noICPRunTillLevel,
								settings->depthTrackerICPThreshold, lowLevelEngine),
							new ITMRenTracker_CPU<TVoxel, TIndex>(imgSize_d, settings->noICPRunTillLevel, lowLevelEngine, denseMapper->getScene())
						};
						break;
					default: break;
					}
#endif
					break;
				default: break;
				}
			}

			virtual ~ITMTrackerCollection(void)
			{
				for (int i = 0; i < noTrackers; i++)
					delete trackers[i];

				delete trackers;
			}

			ITMTrackingState *BuildTrackingState()
			{
				return new ITMTrackingState(trackedImageSize, memoryType);
			}

			static Vector2i GetTrackedImageSize(const ITMLibSettings *settings, const Vector2i& imgSize_rgb, const Vector2i& imgSize_d)
			{
				return settings->trackerType == ITMLibSettings::TRACKER_COLOR ? imgSize_rgb : imgSize_d;
			}

			void TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
			{
				for (int i = 0; i < noTrackers; i++) trackers[i]->TrackCamera(trackingState, view);
			}

			template <typename TVoxel, typename TIndex>
			void RenderWorld(ITMDenseMapper<TVoxel, TIndex> *denseMapper, ITMTrackingState *trackingState, const ITMView *view)
			{
				switch (trackerType)
				{
				case ITMLibSettings::TRACKER_ICP:
				case ITMLibSettings::TRACKER_REN:
				case ITMLibSettings::TRACKER_IMU:
					denseMapper->GetICPMaps(trackingState->pose_d, &(view->calib->intrinsics_d), view, trackingState);
					break;
				case ITMLibSettings::TRACKER_COLOR:
					ITMPose pose_rgb(view->calib->trafo_rgb_to_depth.calib_inv * trackingState->pose_d->GetM());
					denseMapper->GetPointCloud(&pose_rgb, &(view->calib->intrinsics_rgb), view, trackingState, skipPoints);
					break;
				}
			}
		};
	}
}
