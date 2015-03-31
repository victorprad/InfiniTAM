// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMScene.h"
#include "ITMRenderState.h"
#include "ITMTrackingState.h"
#include "../Utils/ITMLibSettings.h"
#include "../Engine/ITMTrackingController.h"
#include "../Engine/ITMVisualisationEngine.h"

namespace ITMLib {
	struct ITMPoseConstraint
	{
	public:
		ITMPoseConstraint(void);

		void AddObservation(const Matrix4f & relative_pose);
		Matrix4f GetAccumulatedInfo(void) const;
	private:
		Matrix4f accu_poses;
		int accu_num;
	};

	template<class TVoxel,class TIndex>
	class ITMLocalScene
	{
	public:
		ITMScene<TVoxel,TIndex> *scene;
		ITMRenderState *renderState;
		ITMTrackingState *trackingState;

		ITMLocalScene(const ITMLibSettings *settings, const IITMVisualisationEngine *visualisationEngine, const ITMTrackingController *trackingController, const Vector2i & trackedImageSize)
		{
			scene = new ITMScene<TVoxel,TIndex>(&(settings->sceneParams), settings->useSwapping, settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU);
			renderState = visualisationEngine->CreateRenderState(scene, trackedImageSize);
			trackingState = trackingController->BuildTrackingState(trackedImageSize);
		}
		~ITMLocalScene(void)
		{
			delete scene;
			delete renderState;
			delete trackingState;
		}
	};
}

