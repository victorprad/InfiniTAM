// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#ifndef __METALC__

#include "../Interface/ITMExtendedTracker.h"

namespace ITMLib
{
    class ITMExtendedTracker_Metal : public ITMExtendedTracker
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
        ITMExtendedTracker_Metal(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
                                 float terminationThreshold, float failureDetectorThreshold, float viewFrustum_min, float viewFrustum_max,
                                 float tukeyCutOff, float framesToSkip, float framesToWeight, const ITMLowLevelEngine *lowLevelEngine);
        ~ITMExtendedTracker_Metal(void);
    };
}

#endif

#if (defined __OBJC__) || (defined __METALC__)

struct ExtendedTrackerOneLevel_rt_Params
{
    Matrix4f approxInvPose;
    Matrix4f scenePose;
    Vector4f sceneIntrinsics;
    Vector4f viewIntrinsics;
    Vector4f others1;
    Vector4f others2;
    Vector4f others3;
    Vector2i sceneImageSize;
    Vector2i viewImageSize;
};

#endif
