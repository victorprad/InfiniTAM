// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#ifndef __METALC__

#include "../../ITMDepthTracker.h"

namespace ITMLib
{
	namespace Engine
	{
		class ITMDepthTracker_Metal : public ITMDepthTracker
		{
        private:
            Vector2i allocImgSize;
            
            float *ATb_metal;
            float *ATA_metal;
            float *noValidPoints_metal;
            
            void *ATb_metal_mb;
            void *ATA_metal_mb;
            void *noValidPoints_metal_mb;
		protected:
			int ComputeGandH(ITMSceneHierarchyLevel *sceneHierarchyLevel, ITMTemplatedHierarchyLevel<ITMFloatImage> *viewHierarchyLevel,
				Matrix4f approxInvPose, Matrix4f imagePose, bool rotationOnly);

		public:
			 ITMDepthTracker_Metal(Vector2i imgSize, int noHierarchyLevels, int noRotationOnlyLevels, int noICPRunTillLevel, float distThresh, ITMLowLevelEngine *lowLevelEngine);
			~ITMDepthTracker_Metal(void);
		};
	}
}

#endif

struct DepthTrackerOneLevel_g_rg_Params
{
    Matrix4f approxInvPose;
    Matrix4f scenePose;
    Vector4f sceneIntrinsics;
    Vector4f viewIntrinsics;
    Vector4f others;
    Vector2i sceneImageSize;
    Vector2i viewImageSize;
};