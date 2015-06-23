// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <math.h>

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMScene.h"
#include "../Objects/ITMMesh.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		template<class TVoxel, class TIndex>
		class ITMMeshingEngine
		{
		public:
			virtual void MeshScene(ITMMesh *mesh, const ITMScene<TVoxel,TIndex> *scene) = 0;

			ITMMeshingEngine(void) { }
			virtual ~ITMMeshingEngine(void) { }
		};
	}
}
