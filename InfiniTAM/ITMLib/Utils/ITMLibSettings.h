// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMSceneParams.h"
#include "ITMSurfelSceneParams.h"
#include "../../ORUtils/MemoryDeviceType.h"

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

		typedef enum
		{
			FAILUREMODE_RELOCALISE,
			FAILUREMODE_IGNORE,
			FAILUREMODE_STOP_INTEGRATION
		} FailureMode;
        
		typedef enum
		{
			SWAPPINGMODE_DISABLED,
			SWAPPINGMODE_ENABLED,
			SWAPPINGMODE_DELETE
		} SwappingMode;

		/// Select the type of device to use
		DeviceType deviceType;

		bool useApproximateRaycast;

		bool useBilateralFilter;

		/// For ITMColorTracker: skip every other point in energy function evaluation.
		bool skipPoints;

		bool createMeshingEngine;
        
		FailureMode behaviourOnFailure;
		SwappingMode swappingMode;

		const char *trackerConfig;

		/// Further, scene specific parameters such as voxel size
		ITMSceneParams sceneParams;
		ITMSurfelSceneParams surfelSceneParams;

		ITMLibSettings(void);
		~ITMLibSettings(void) {}

		// Suppress the default copy constructor and assignment operator
		ITMLibSettings(const ITMLibSettings&);
		ITMLibSettings& operator=(const ITMLibSettings&);

		MemoryDeviceType GetMemoryType() const;
	};
}
