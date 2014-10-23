// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Objects/ITMSceneParams.h"

namespace ITMLib
{
	namespace Objects
	{
		class ITMLibSettings
		{
		public:
			/// Use GPU or run the code on the CPU instead.
			bool useGPU;

			/// Enables swapping between host and device.
			bool useSwapping;

			/// Tracker types
			typedef enum {
				//! Identifies a tracker based on colour image
				TRACKER_COLOR,
				//! Identifies a tracker based on depth image
				TRACKER_ICP,
				//! Identifies a tracker based on depth image (Ren et al, 2012)
				TRACKER_REN
			} TrackerType;
			/// Select the type of tracker to use
			TrackerType trackerType;

			/// Number of resolution levels for the tracker.
			int noHierarchyLevels;

			/// Number of resolution levels to track only rotation instead of full SE3.
			int noRotationOnlyLevels;
			
			/// Run ICP till # Hierarchy level, then switch to ITMRenTracker for local refinement.
			int noICPRunTillLevel;

			/// For ITMColorTracker: skip every other point in energy function evaluation.
			bool skipPoints;

			/// For ITMDepthTracker: ICP distance threshold
			float depthTrackerICPThreshold;

			/// Further, scene specific parameters such as voxel size
			ITMLib::Objects::ITMSceneParams sceneParams;

			ITMLibSettings(void);
			~ITMLibSettings(void) { }
		};
	}
}
