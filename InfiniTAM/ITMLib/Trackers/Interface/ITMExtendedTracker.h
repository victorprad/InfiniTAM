// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMTracker.h"
#include "../../Engines/LowLevel/Interface/ITMLowLevelEngine.h"
#include "../../Objects/Tracking/ITMImageHierarchy.h"
#include "../../Objects/Tracking/ITMDepthHierarchyLevel.h"
#include "../../Objects/Tracking/ITMIntensityHierarchyLevel.h"
#include "../../Objects/Tracking/ITMSceneHierarchyLevel.h"
#include "../../Objects/Tracking/ITMTemplatedHierarchyLevel.h"
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
		static const int MIN_VALID_POINTS_DEPTH;
		static const int MIN_VALID_POINTS_RGB;

		const ITMLowLevelEngine *lowLevelEngine;
		ITMImageHierarchy<ITMSceneHierarchyLevel> *sceneHierarchy;
		ITMImageHierarchy<ITMDepthHierarchyLevel> *viewHierarchy_Depth;
		ITMImageHierarchy<ITMIntensityHierarchyLevel> *viewHierarchy_Intensity;
		ITMImageHierarchy<ITMTemplatedHierarchyLevel<ITMFloat4Image> > *reprojectedPointsHierarchy;
		ITMImageHierarchy<ITMTemplatedHierarchyLevel<ITMFloatImage> > *projectedIntensityHierarchy;

		ITMTrackingState *trackingState;
		const ITMView *view;

		int *noIterationsPerLevel;

		float terminationThreshold;

		float colourWeight;

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
		float *colourThresh;

		int currentLevelId;
		TrackerIterationType currentIterationType;

		Matrix4f scenePose;
		Matrix4f depthToRGBTransform;
		ITMSceneHierarchyLevel *sceneHierarchyLevel_Depth;
		ITMDepthHierarchyLevel *viewHierarchyLevel_Depth;
		ITMIntensityHierarchyLevel *viewHierarchyLevel_Intensity;
		ITMTemplatedHierarchyLevel<ITMFloat4Image> *reprojectedPointsLevel;
		ITMTemplatedHierarchyLevel<ITMFloatImage > *projectedIntensityLevel;

		bool useColour;
		bool useDepth;

		float minColourGradient;
		float viewFrustum_min, viewFrustum_max;
		float tukeyCutOff;
		int framesToSkip, framesToWeight;
		int framesProcessed;

		virtual int ComputeGandH_Depth(float &f, float *nabla, float *hessian, Matrix4f approxInvPose) = 0;
		virtual int ComputeGandH_RGB(float &f, float *nabla, float *hessian, Matrix4f approxPose) = 0;
		virtual void ProjectCurrentIntensityFrame(ITMFloat4Image *points_out,
												  ITMFloatImage *intensity_out,
												  const ITMFloatImage *intensity_in,
												  const ITMFloatImage *depth_in,
												  const Vector4f &intrinsics_depth,
												  const Vector4f &intrinsics_rgb,
												  const Matrix4f &scenePose) = 0;

	public:
		void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

		bool requiresColourRendering() const { return false; }
		bool requiresDepthReliability() const { return true; }
		bool requiresPointCloudRendering() const { return true; }

		void SetupLevels(int numIterCoarse, int numIterFine, float spaceThreshCoarse, float spaceThreshFine, float colourThreshCoarse, float colourThreshFine);

		ITMExtendedTracker(Vector2i imgSize_d,
						   Vector2i imgSize_rgb,
						   bool useDepth,
						   bool useColour,
						   float colourWeight,
						   TrackerIterationType *trackingRegime,
						   int noHierarchyLevels,
						   float terminationThreshold,
						   float failureDetectorThreshold,
						   float viewFrustum_min,
						   float viewFrustum_max,
						   float minColourGradient,
						   float tukeyCutOff,
						   int framesToSkip,
						   int framesToWeight,
						   const ITMLowLevelEngine *lowLevelEngine,
						   MemoryDeviceType memoryType
						   );
		virtual ~ITMExtendedTracker(void);
	};
}
