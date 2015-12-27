// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Objects/ITMRenderState.h"
#include "../../Objects/ITMView.h"
#include "../../Objects/Scene/ITMScene.h"

namespace ITMLib
{
	/** \brief
	Interface to engines that swap data in and out of the
	fairly limited GPU memory to some large scale storage
	space.
	*/
	template<class TVoxel, class TIndex>
	class ITMSwappingEngine
	{
	public:
		virtual void IntegrateGlobalIntoLocal(ITMScene<TVoxel, TIndex> *scene, ITMRenderState *renderState) = 0;

		virtual void SaveToGlobalMemory(ITMScene<TVoxel, TIndex> *scene, ITMRenderState *renderState) = 0;

		virtual ~ITMSwappingEngine(void) { }
	};
}
