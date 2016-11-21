// Copyright 2016 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMMultiVisualisationEngine.h"

namespace ITMLib 
{
	template<class TVoxel, class TIndex>
	class ITMMultiVisualisationEngine_CPU : public ITMMultiVisualisationEngine<TVoxel, TIndex>
	{
	public:
		ITMMultiVisualisationEngine_CPU(void) {}
		~ITMMultiVisualisationEngine_CPU(void) {}

		ITMRenderState* CreateRenderState(const ITMScene<TVoxel, TIndex> *scene, const Vector2i & imgSize) const;

		void PrepareRenderState(const ITMMultiSceneManager_instance<TVoxel, TIndex> & sceneManager, ITMRenderState *state);

		void CreateExpectedDepths(const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;

		void RenderImage(const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState, ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type) const;
	};
}

