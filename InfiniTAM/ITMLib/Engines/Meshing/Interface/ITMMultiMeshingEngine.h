// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMMeshingEngine.h"
#include "../../MultiScene/ITMMultiSceneManager.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMMultiMeshingEngine
	{
	public:
		virtual ~ITMMultiMeshingEngine(void) {}

		virtual void MeshScene(ITMMesh *mesh, const ITMMultiSceneManager_instance<TVoxel, TIndex> & sceneManager) = 0;
	};
}