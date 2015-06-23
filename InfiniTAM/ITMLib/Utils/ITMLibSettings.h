// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Objects/ITMSceneParams.h"
#include "../Engine/ITMTracker.h"

namespace ITMLib
{
	namespace Objects
	{
		class ITMLibSettings
		{
		public:
			/// The device used to run the DeviceAgnostic code
			typedef enum {
				DEVICE_CPU,
				DEVICE_CUDA,
				DEVICE_METAL
			} DeviceType;

			/// Select the type of device to use
			DeviceType deviceType;

			/// Enables swapping between host and device.
			bool useSwapping;

			bool useApproximateRaycast;

			bool useBilateralFilter;

			bool modelSensorNoise;

			/// Tracker types
			typedef enum {
				//! Identifies a tracker based on colour image
				TRACKER_COLOR,
				//! Identifies a tracker based on depth image
				TRACKER_ICP,
				//! Identifies a tracker based on depth image (Ren et al, 2012)
				TRACKER_REN,
				//! Identifies a tracker based on depth image and IMU measurement
				TRACKER_IMU,
				//! Identifies a tracker that use weighted ICP only on depth image
				TRACKER_WICP
			} TrackerType;

			/// Select the type of tracker to use
			TrackerType trackerType;

			/// The tracking regime used by the tracking controller
			TrackerIterationType *trackingRegime;

			/// The number of levels in the trackingRegime
			int noHierarchyLevels;
			
			/// Run ICP till # Hierarchy level, then switch to ITMRenTracker for local refinement.
			int noICPRunTillLevel;

			/// For ITMColorTracker: skip every other point in energy function evaluation.
			bool skipPoints;

			/// For ITMDepthTracker: ICP distance threshold
			float depthTrackerICPThreshold;

			/// For ITMDepthTracker: ICP iteration termination threshold
			float depthTrackerTerminationThreshold;

			/// Further, scene specific parameters such as voxel size
			ITMLib::Objects::ITMSceneParams sceneParams;

			ITMLibSettings(void);
			~ITMLibSettings(void);

			// Suppress the default copy constructor and assignment operator
			ITMLibSettings(const ITMLibSettings&);
			ITMLibSettings& operator=(const ITMLibSettings&);
		};
	}
}
