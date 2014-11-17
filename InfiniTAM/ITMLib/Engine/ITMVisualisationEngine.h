// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMImage.h"
#include "../Objects/ITMScene.h"
#include "../Objects/ITMView.h"
#include "../Objects/ITMTrackingState.h"
#include "../Objects/ITMVisualisationState.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		/** \brief
		    Interface to engines helping with the visualisation of
		    the results from the rest of the library.

		    This is also used internally to get depth estimates for the
		    raycasting done for the trackers. The basic idea there is
		    to project down a scene of 8x8x8 voxel
		    blocks and look at the bounding boxes. The projection
		    provides an idea of the possible depth range for each pixel
		    in an image, which can be used to speed up raycasting
		    operations.
		*/
		template<class TVoxel, class TIndex>
		class ITMVisualisationEngine
		{
		public:
			virtual ~ITMVisualisationEngine(void) {}

			static void DepthToUchar4(ITMUChar4Image *dst, ITMFloatImage *src);

			/** This will allocate an appropriate internal state
			    object (GPU, CPU, wherever) that can then be used
			    by the rest of the engine.
			*/
			virtual ITMVisualisationState* allocateInternalState(const Vector2i & imgSize) = 0;

			/** Given a scene, pose and intrinsics, compute the
			    visible subset of the scene and store it in an
			    appropriate visualisation state object, created
			    previously using allocateInternalState().
			*/
			virtual void FindVisibleBlocks(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMVisualisationState *state) = 0;
			/** Given scene, pose and intrinsics, create an estimate
			    of the minimum and maximum depths at each pixel of
			    an image.
			*/
			virtual void CreateExpectedDepths(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, 
				const ITMIntrinsics *intrinsics, ITMFloat2Image *minmaxImg, const ITMVisualisationState *state = NULL) = 0;

			/** This will render an image using raycasting. */
			virtual void RenderImage(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMVisualisationState *state, ITMUChar4Image *outputImage, bool useColour) = 0;

			/** Create a point cloud as required by the
			    ITMLib::Engine::ITMColorTracker classes.
			*/
			virtual void CreatePointCloud(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, bool skipPoints) = 0;

			/** Create an image of reference points and normals as
			    required by the ITMLib::Engine::ITMDepthTracker
			    classes.
			*/
			virtual void CreateICPMaps(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState) = 0;
		};
	}
}
