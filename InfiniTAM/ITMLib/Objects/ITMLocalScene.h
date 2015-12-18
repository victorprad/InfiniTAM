// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <map>

#include "ITMScene.h"
#include "ITMRenderState.h"
#include "../Trackers/Base/ITMTrackingState.h"
#include "../Utils/ITMLibSettings.h"
#include "../Visualisation/Interface/ITMVisualisationEngine.h"

namespace ITMLib {
	struct ITMPoseConstraint
	{
	public:
		ITMPoseConstraint(void);

		void AddObservation(const ITMPose & relative_pose, int weight = 1);
		ITMPose GetAccumulatedInfo(void) const { return accu_poses; }
		int GetNumAccumulatedInfo(void) const { return accu_num; }
	private:
		ITMPose accu_poses;
		int accu_num;
	};

	template<class TVoxel,class TIndex>
	class ITMLocalScene
	{
	public:
		ITMScene<TVoxel,TIndex> *scene;
		ITMRenderState *renderState;
		ITMTrackingState *trackingState;
		std::map<int,ITMPoseConstraint> relations;

		ITMLocalScene(const ITMLibSettings *settings, const ITMVisualisationEngine<TVoxel,TIndex> *visualisationEngine, const Vector2i & trackedImageSize)
		{
			MemoryDeviceType memoryType = settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
			scene = new ITMScene<TVoxel,TIndex>(&settings->sceneParams, settings->useSwapping, memoryType);
			renderState = visualisationEngine->CreateRenderState(scene, trackedImageSize);
			trackingState = new ITMTrackingState(trackedImageSize, memoryType);
		}
		~ITMLocalScene(void)
		{
			delete scene;
			delete renderState;
			delete trackingState;
		}

		/** Check whether this is a new scene that has not been
		    completely localised relative to any others.
		*/
/*		bool isNewScene(void) const
		{
			return (relations.size() == 0);
		}*/
	};
}

