// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

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

			float ATA_host[6 * 6];
			float ATb_host[6];
			float step[6];
			float f;

			void PrepareForEvaluation();
			void SetEvaluationParams(int levelId);

			void ComputeSingleStep(float *step, float *ATA, float *ATb, bool shortIteration);
			void ApplyDelta(const Matrix4f & para_old, const float *delta, Matrix4f & para_new) const;

			void SetEvaluationData(ITMTrackingState *trackingState, const ITMView *view);
		protected:
			float distThresh;

			int levelId;
			TrackerIterationType iterationType;

			Matrix4f scenePose;
			ITMSceneHierarchyLevel *sceneHierarchyLevel;
			ITMTemplatedHierarchyLevel<ITMFloatImage> *viewHierarchyLevel;

			virtual int ComputeGandH(float &f, float *ATb_host, float *ATA_host, Matrix4f approxInvPose) = 0;

		public:
			void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

			ITMDepthTracker(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
				const ITMLowLevelEngine *lowLevelEngine, MemoryDeviceType memoryType);
			virtual ~ITMDepthTracker(void);
		};
	}
}
