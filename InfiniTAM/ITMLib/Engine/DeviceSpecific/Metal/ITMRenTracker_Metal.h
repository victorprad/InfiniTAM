// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMRenTracker.h"

namespace ITMLib
{
	namespace Engine
	{
		template<class TVoxel, class TIndex>
		class ITMRenTracker_Metal : public ITMRenTracker<TVoxel,TIndex>
		{
		protected:
			void F_oneLevel(float *f, Matrix4f invM);
			void G_oneLevel(float *gradient, float *hessian, Matrix4f invM) const;

			void UnprojectDepthToCam(ITMFloatImage *depth, ITMFloat4Image *upPtCloud, const Vector4f &intrinsic);

		public:
			ITMRenTracker_Metal(Vector2i imgSize, int noHierarchyLevels, ITMLowLevelEngine *lowLevelEngine, ITMScene<TVoxel, TIndex> *scene);
			~ITMRenTracker_Metal(void);
		};
	}
}
