// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMImageHierarchy.h"
#include "../Objects/ITMTemplatedHierarchyLevel.h"

#include "../Engine/ITMTracker.h"
#include "../Engine/ITMLowLevelEngine.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		/** Base class for engine performing SDF based depth tracking.
		*/
		template<class TVoxel, class TIndex>
		class ITMRenTracker : public ITMTracker
		{
		private:
			ITMTrackingState *trackingState; 
			ITMLowLevelEngine *lowLevelEngine;

			ITMFloatImage *tempImage1, *tempImage2;

			const ITMView *view;

			int *noIterationsPerLevel;

			
			
			void PrepareForEvaluation(const ITMView *view);

		protected:
			ITMScene<TVoxel, TIndex> *scene;
			ITMImageHierarchy<ITMTemplatedHierarchyLevel<ITMFloat4Image> > *viewHierarchy;

			int levelId;
			bool rotationOnly;

			float ATA_host[6 * 6];
			float ATb_host[6];

			virtual void F_oneLevel(float *f, Matrix4f invM) = 0;
			virtual void G_oneLevel(float *gradient, float *hessian, Matrix4f invM) const = 0;

			virtual void UnprojectDepthToCam(ITMFloatImage *depth, ITMFloat4Image *upPtCloud, const Vector4f &intrinsic) = 0;

		public:

			void applyDelta(const ITMPose & para_old, const float *delta, ITMPose & para_new) const;
			int numParameters(void) const { return 6; }

			class EvaluationPoint
			{
			public:
				float f(void) { return cacheF; }
				const float* nabla_f(void) { if (cacheNabla == NULL) computeGradients(false); return cacheNabla; }

				const float* hessian_GN(void) { if (cacheHessian == NULL) computeGradients(true); return cacheHessian; }
				const ITMPose & getParameter(void) const { return *mPara; }

				EvaluationPoint(ITMPose *pos, const ITMRenTracker *f_parent);
				~EvaluationPoint(void)
				{
					delete mPara;
					if (cacheNabla!=NULL) delete[] cacheNabla;
					if (cacheHessian!=NULL) delete[] cacheHessian;
				}

			protected:
				void computeGradients(bool requiresHessian);

				ITMPose *mPara;
				const ITMRenTracker *mParent;

				float cacheF;
				float *cacheNabla;
				float *cacheHessian;
			};

			EvaluationPoint* evaluateAt(ITMPose *para) const
			{
				return new EvaluationPoint(para, this);
			}

			void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

			ITMRenTracker(Vector2i imgSize, int noHierarchyLevels, ITMLowLevelEngine *lowLevelEngine, ITMScene<TVoxel,TIndex> *scene, bool useGPU);
			virtual ~ITMRenTracker(void);
		};
	}
}
