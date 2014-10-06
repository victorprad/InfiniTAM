// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMScene.h"
#include "../Objects/ITMView.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		/** \brief
		    Interface to engines that swap data in and out of the
		    fairly limited GPU memory to some large scale storage
		    space.
		*/
		template<class TVoxel,class TIndex>
		class ITMSwappingEngine
		{
		public:
			virtual void IntegrateGlobalIntoLocal(ITMScene<TVoxel,TIndex> *scene, ITMView *view) = 0;

			virtual void SaveToGlobalMemory(ITMScene<TVoxel,TIndex> *scene, ITMView *view) = 0;

			virtual ~ITMSwappingEngine(void) { }
		};
	}
}
