// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <math.h>

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMScene.h"
#include "../Objects/ITMView.h"
#include "../Objects/ITMTrackingState.h"
#include "../Objects/ITMRenderState.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		/** \brief
		    Interface to engines implementing the main KinectFusion
		    depth integration process.

		    These classes basically manage
		    an ITMLib::Objects::ITMScene and fuse new image information
		    into them.
		*/
		template<class TVoxel, class TIndex>
		class ITMSceneReconstructionEngine
		{
		public:
			/** Clear and reset a scene to set up a new empty
			    one.
			*/
			virtual void ResetScene(ITMScene<TVoxel, TIndex> *scene) = 0;

			/** Given a view with a new depth image, compute the
			    visible blocks, allocate them and update the hash
			    table so that the new image data can be integrated.
			*/
			virtual void AllocateSceneFromDepth(ITMScene<TVoxel,TIndex> *scene, const ITMView *view, const ITMTrackingState *trackingState,
				const ITMRenderState *renderState, bool onlyUpdateVisibleList = false) = 0;

			/** Update the voxel blocks by integrating depth and
			    possibly colour information from the given view.
			*/
			virtual void IntegrateIntoScene(ITMScene<TVoxel,TIndex> *scene, const ITMView *view, const ITMTrackingState *trackingState,
				const ITMRenderState *renderState) = 0;

			ITMSceneReconstructionEngine(void) { }
			virtual ~ITMSceneReconstructionEngine(void) { }
		};
	}
}
