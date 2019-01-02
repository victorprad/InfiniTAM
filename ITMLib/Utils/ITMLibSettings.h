// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMSceneParams.h"
#include "ITMSurfelSceneParams.h"
#include "../../ORUtils/DeviceType.h"
#include "../../ORUtils/MemoryDeviceType.h"

namespace ITMLib
{
	class ITMLibSettings
	{
	public:
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

		typedef enum
		{
			LIBMODE_BASIC,
			LIBMODE_BASIC_SURFELS,
			LIBMODE_LOOPCLOSURE
		} LibMode;

		/// Select the type of device to use
		ORUtils::DeviceType deviceType;

		bool useApproximateRaycast;

		bool useBilateralFilter;

		/// For ITMColorTracker: skip every other point in energy function evaluation.
		bool skipPoints;

		bool createMeshingEngine;
        
		FailureMode behaviourOnFailure;
		SwappingMode swappingMode;
		LibMode libMode;

		const char *trackerConfig;

		/// Further, scene specific parameters such as voxel size
		ITMSceneParams sceneParams;
		ITMSurfelSceneParams surfelSceneParams;

		ITMLibSettings(void);
		virtual ~ITMLibSettings(void) {}

		// Suppress the default copy constructor and assignment operator
		ITMLibSettings(const ITMLibSettings&);
		ITMLibSettings& operator=(const ITMLibSettings&);

		MemoryDeviceType GetMemoryType() const;
	};
}
