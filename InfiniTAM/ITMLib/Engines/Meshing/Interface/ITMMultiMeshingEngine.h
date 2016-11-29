// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMMeshingEngine.h"
#include "../../../Objects/Meshing/ITMMesh.h"
#include "../../MultiScene/ITMMapGraphManager.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMMultiMeshingEngine
	{
	public:
		virtual ~ITMMultiMeshingEngine(void) {}

		virtual void MeshScene(ITMMesh *mesh, const ITMVoxelMapGraphManager<TVoxel, TIndex> & sceneManager) = 0;
	};
}