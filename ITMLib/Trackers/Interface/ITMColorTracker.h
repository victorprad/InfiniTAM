// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

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
			const ORUtils::SE3Pose & getParameter(void) const { return *mPara; }

			EvaluationPoint(ORUtils::SE3Pose *pos, const ITMColorTracker *f_parent);
			~EvaluationPoint(void)
			{
				delete mPara;
				if (cacheNabla != NULL) delete[] cacheNabla;
				if (cacheHessian != NULL) delete[] cacheHessian;
			}

			int getNumValidPoints(void) const { return mValidPoints; }

		protected:
			void computeGradients(bool requiresHessian);

			ORUtils::SE3Pose *mPara;
			const ITMColorTracker *mParent;

			float cacheF;
			float *cacheNabla;
			float *cacheHessian;
			int mValidPoints;
		};

		EvaluationPoint* evaluateAt(ORUtils::SE3Pose *para) const
		{
			return new EvaluationPoint(para, this);
		}

		int numParameters(void) const { return (iterationType == TRACKER_ITERATION_ROTATION) ? 3 : 6; }

		virtual int F_oneLevel(float *f, ORUtils::SE3Pose *pose) = 0;
		virtual void G_oneLevel(float *gradient, float *hessian, ORUtils::SE3Pose *pose) const = 0;

		void ApplyDelta(const ORUtils::SE3Pose & para_old, const float *delta, ORUtils::SE3Pose & para_new) const;

		void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

		bool requiresColourRendering() const { return true; }
		bool requiresDepthReliability() const { return false; }
		bool requiresPointCloudRendering() const { return true; }

		ITMColorTracker(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
			const ITMLowLevelEngine *lowLevelEngine, MemoryDeviceType memoryType);
		virtual ~ITMColorTracker(void);
	};
}
