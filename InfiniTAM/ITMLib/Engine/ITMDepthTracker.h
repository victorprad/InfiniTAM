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
			ITMLowLevelEngine *lowLevelEngine;
			ITMImageHierarchy<ITMSceneHierarchyLevel> *sceneHierarchy;
			ITMImageHierarchy<ITMTemplatedHierarchyLevel<ITMFloatImage> > *viewHierarchy;

			ITMTrackingState *trackingState; const ITMView *view;

			int *noIterationsPerLevel;
			int noICPLevel;

			int levelId;
			bool rotationOnly;

			void PrepareForEvaluation();
			void SetEvaluationParams(int levelId);

			void ComputeSingleStep(float *step, float *ATA, float *ATb, bool rotationOnly);
			Matrix4f ApplySingleStep(Matrix4f approxInvPose, float *step);

			void SetEvaluationData(ITMTrackingState *trackingState, const ITMView *view);
		protected:
			float ATA_host[6 * 6];
			float ATb_host[6];
			float step[6];
			float distThresh;

			virtual void ChangeIgnorePixelToZero(ITMFloatImage *image) = 0;
			virtual int ComputeGandH(ITMSceneHierarchyLevel *sceneHierarchyLevel, ITMTemplatedHierarchyLevel<ITMFloatImage> *viewHierarchyLevel,
				Matrix4f approxInvPose, Matrix4f imagePose, bool rotationOnly) = 0;

		public:
			void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

			ITMDepthTracker(Vector2i imgSize, int noHierarchyLevels, int noRotationOnlyLevels, int noICPRunTillLevel, float distThresh, ITMLowLevelEngine *lowLevelEngine, bool useGPU);
			virtual ~ITMDepthTracker(void);
		};
	}
}
