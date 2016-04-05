// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMTracker.h"
#include "../../Engines/LowLevel/Interface/ITMLowLevelEngine.h"
#include "../../Objects/Tracking/ITMImageHierarchy.h"
#include "../../Objects/Tracking/ITMExtendedHierarchyLevel.h"
#include "../../Objects/Tracking/ITMSceneHierarchyLevel.h"
#include "../../Objects/Tracking/TrackerIterationType.h"

#include "../../../ORUtils/HomkerMap.h"
#include "../../../ORUtils/SVMClassifier.h"

namespace ITMLib
{
	/** Base class for engine performing ICP based depth tracking.
	    A typical example would be the original KinectFusion
	    tracking algorithm.
	*/
	class ITMExtendedTracker : public ITMTracker
	{
	private:
		const ITMLowLevelEngine *lowLevelEngine;
		ITMImageHierarchy<ITMSceneHierarchyLevel> *sceneHierarchy;
		ITMImageHierarchy<ITMExtendHierarchyLevel> *viewHierarchy;

		ITMTrackingState *trackingState; const ITMView *view;

		int *noIterationsPerLevel;

		float terminationThreshold;

		void PrepareForEvaluation();
		void SetEvaluationParams(int levelId);

		void ComputeDelta(float *delta, float *nabla, float *hessian, bool shortIteration) const;
		void ApplyDelta(const Matrix4f & para_old, const float *delta, Matrix4f & para_new) const;
		bool HasConverged(float *step) const;

		void SetEvaluationData(ITMTrackingState *trackingState, const ITMView *view);

		void UpdatePoseQuality(int noValidPoints_old, float *hessian_good, float f_old);

		ORUtils::HomkerMap *map;
		ORUtils::SVMClassifier *svmClassifier;
		Vector4f mu, sigma;
	protected:
		float *spaceThresh;

		int levelId;
		TrackerIterationType iterationType;

		Matrix4f scenePose;
		ITMSceneHierarchyLevel *sceneHierarchyLevel;
		ITMExtendHierarchyLevel *viewHierarchyLevel;

		int currentFrameNo;

		virtual int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose) = 0;

	public:
		void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

		bool requiresColourRendering(void) const { return false; }
		bool requiresDepthReliability(void) const { return true; }

		void SetupLevels(int numIterCoarse, int numIterFine, float spaceThreshCoarse, float spaceThreshFine);

		ITMExtendedTracker(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
			float terminationThreshold, float failureDetectorThreshold, 
			const ITMLowLevelEngine *lowLevelEngine, MemoryDeviceType memoryType);
		virtual ~ITMExtendedTracker(void);
	};
}
