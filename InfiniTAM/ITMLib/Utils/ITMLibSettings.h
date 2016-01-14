// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMSceneParams.h"

namespace ITMLib
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

		/// For ITMColorTracker: skip every other point in energy function evaluation.
		bool skipPoints;

		bool useRelocalisation;
		bool useTrackingFailureDetection;

		float goodTrackingThreshold;
		float poorTrackingThreshold;

		const char *trackerConfig;

		/// Further, scene specific parameters such as voxel size
		ITMLib::ITMSceneParams sceneParams;

		ITMLibSettings(void);
		~ITMLibSettings(void) {}

		// Suppress the default copy constructor and assignment operator
		ITMLibSettings(const ITMLibSettings&);
		ITMLibSettings& operator=(const ITMLibSettings&);
	};
}
