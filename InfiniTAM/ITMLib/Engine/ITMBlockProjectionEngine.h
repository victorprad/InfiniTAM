// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Objects/ITMScene.h"
#include "../Objects/ITMView.h"
#include "../Utils/ITMLibDefines.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		/** \brief
		    Interface for engines to create an estimate of the depth
		    range at each pixel.

		    The basic idea is to project down a scene of 8x8x8 voxel
		    blocks and look at the bounding boxes. The projection
		    provides an idea of the possible depth range for each pixel
		    in an image, which can be used to speed up raycasting
		    operations.
		*/
		template<class TVoxel, class TIndex>
		class ITMBlockProjectionEngine
		{
		public:
			virtual ~ITMBlockProjectionEngine(void) {}

			/** Given scene, pose and intrinsics, create an estimate
			    of the minimum and maximum depths at each pixel of
			    an image.
			*/
			virtual void CreateExpectedDepths(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, 
				const ITMIntrinsics *intrinsics, ITMFloat2Image *minmaxImg) = 0;
		};
	}
}

