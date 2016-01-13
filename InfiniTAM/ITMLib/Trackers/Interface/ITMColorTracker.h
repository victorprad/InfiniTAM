// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMTracker.h"
#include "../../Engines/LowLevel/Interface/ITMLowLevelEngine.h"
#include "../../Objects/Tracking/ITMImageHierarchy.h"
#include "../../Objects/Tracking/ITMViewHierarchyLevel.h"
#include "../../Objects/Tracking/TrackerIterationType.h"

namespace ITMLib
{
	/** Base class for engines performing point based colour
	    tracking. Implementations would typically project down a
	    point cloud into observed images and try to minimize the
	    reprojection error.
	*/
	class ITMColorTracker : public ITMTracker
	{
	private:
		const ITMLowLevelEngine *lowLevelEngine;

		void PrepareForEvaluation(const ITMView *view);

	protected: 
		TrackerIterationType iterationType;
		ITMTrackingState *trackingState; const ITMView *view;
		ITMImageHierarchy<ITMViewHierarchyLevel> *viewHierarchy;
		int levelId;

		int countedPoints_valid;
	public:
		class EvaluationPoint
		{
		public:
			float f(void) { return cacheF; }
			const float* nabla_f(void) { if (cacheNabla == NULL) computeGradients(false); return cacheNabla; }

			const float* hessian_GN(void) { if (cacheHessian == NULL) computeGradients(true); return cacheHessian; }
			const ITMPose & getParameter(void) const { return *mPara; }

			EvaluationPoint(ITMPose *pos, const ITMColorTracker *f_parent);
			~EvaluationPoint(void)
			{
				delete mPara;
				if (cacheNabla != NULL) delete[] cacheNabla;
				if (cacheHessian != NULL) delete[] cacheHessian;
			}

		protected:
			void computeGradients(bool requiresHessian);

			ITMPose *mPara;
			const ITMColorTracker *mParent;

			float cacheF;
			float *cacheNabla;
			float *cacheHessian;
		};

		EvaluationPoint* evaluateAt(ITMPose *para) const
		{
			return new EvaluationPoint(para, this);
		}

		int numParameters(void) const { return (iterationType == TRACKER_ITERATION_ROTATION) ? 3 : 6; }

		virtual void F_oneLevel(float *f, ITMPose *pose) = 0;
		virtual void G_oneLevel(float *gradient, float *hessian, ITMPose *pose) const = 0;

		void ApplyDelta(const ITMPose & para_old, const float *delta, ITMPose & para_new) const;

		void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

		bool requiresColourRendering(void) const { return true; }
		bool requiresDepthReliability(void) const { return false; }

		ITMColorTracker(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
			const ITMLowLevelEngine *lowLevelEngine, MemoryDeviceType memoryType);
		virtual ~ITMColorTracker(void);
	};
}
