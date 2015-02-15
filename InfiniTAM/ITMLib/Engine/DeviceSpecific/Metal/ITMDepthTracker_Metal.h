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
            float *f_metal;
            
            void *ATb_metal_mb;
            void *ATA_metal_mb;
            void *noValidPoints_metal_mb;
            void *f_metal_mb;
		protected:
            int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);

		public:
            ITMDepthTracker_Metal(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
                                  int noICPRunTillLevel, float distThresh, float terminationThreshold, const ITMLowLevelEngine *lowLevelEngine);
			~ITMDepthTracker_Metal(void);
		};
	}
}

#endif

#if (defined __OBJC__) || (defined __METALC__)

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

#endif