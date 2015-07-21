// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMImageHierarchy.h"
#include "../Objects/ITMTemplatedHierarchyLevel.h"
#include "../Objects/ITMSceneHierarchyLevel.h"

#include "../Engine/ITMTracker.h"
#include "../Engine/ITMLowLevelEngine.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		/** Base class for engine performing ICP based depth tracking.
		    A typical example would be the original KinectFusion
		    tracking algorithm.
		*/
		class ITMDepthTracker : public ITMTracker
		{
		private:
			const ITMLowLevelEngine *lowLevelEngine;
			ITMImageHierarchy<ITMSceneHierarchyLevel> *sceneHierarchy;
			ITMImageHierarchy<ITMTemplatedHierarchyLevel<ITMFloatImage> > *viewHierarchy;

			ITMTrackingState *trackingState; const ITMView *view;

			int *noIterationsPerLevel;
			int noICPLevel;

			float terminationThreshold;

			void PrepareForEvaluation();
			void SetEvaluationParams(int levelId);

			void ComputeDelta(float *delta, float *nabla, float *hessian, bool shortIteration) const;
			void ApplyDelta(const Matrix4f & para_old, const float *delta, Matrix4f & para_new) const;
			bool HasConverged(float *step) const;

			void SetEvaluationData(ITMTrackingState *trackingState, const ITMView *view);
		protected:
			float *distThresh;

			int levelId;
			TrackerIterationType iterationType;

			Matrix4f scenePose;
			ITMSceneHierarchyLevel *sceneHierarchyLevel;
			ITMTemplatedHierarchyLevel<ITMFloatImage> *viewHierarchyLevel;

			virtual int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose) = 0;

		public:
			void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

			ITMDepthTracker(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
				float terminationThreshold, const ITMLowLevelEngine *lowLevelEngine, MemoryDeviceType memoryType);
			virtual ~ITMDepthTracker(void);
		};
	}
}
