// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include <metal_stdlib>

using namespace metal;

#include "../Shared/ITMExtendedTracker_Shared.h"
#include "ITMExtendedTracker_Metal.h"
#include "../../ITMLibDefines.h"
#include "../../Objects/Tracking/TrackerIterationType.h"

inline float rho(float r, float huber_b)
{
    float tmp = fabs(r) - huber_b;
    tmp = MAX(tmp, 0.0f);
    return r*r - tmp*tmp;
}

inline float rho_deriv(float r, float huber_b)
{
    return 2.0f * CLAMP(r, -huber_b, huber_b);
}

inline float rho_deriv2(float r, float huber_b)
{
    if (fabs(r) < huber_b) return 2.0f;
    return 0.0f;
}

kernel void extendedTrackerOneLevel_g_rt_device(DEVICEPTR(float) *noValidPoints                             [[ buffer(0) ]],
                                                DEVICEPTR(float) *f                                         [[ buffer(1) ]],
                                                DEVICEPTR(float) *ATA                                       [[ buffer(2) ]],
                                                DEVICEPTR(float) *ATb                                       [[ buffer(3) ]],
                                                const CONSTPTR(float) *depth                                [[ buffer(4) ]],
                                                const CONSTPTR(Vector4f) *pointsMap                         [[ buffer(5) ]],
                                                const CONSTPTR(Vector4f) *normalsMap                        [[ buffer(6) ]],
                                                const CONSTPTR(ExtendedTrackerOneLevel_rt_Params) *params   [[ buffer(7) ]],
                                                uint2 threadIdx                                             [[ thread_position_in_threadgroup ]],
                                                uint2 blockIdx                                              [[ threadgroup_position_in_grid ]],
                                                uint2 blockDim                                              [[ threads_per_threadgroup ]])
{
    int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;
    
    float localNabla[6], localHessian[21], localF; bool isValidPoint = false;
    float b, A[6];
    
    float spaceThresh = params->others1.x;
    int iterationType = (int)params->others1.y;
    int pointsPerThread = (int)params->others1.z;
    float viewFrustum_min = params->others1.w;
    float viewFrustum_max = params->others2.x;
    int tukeyCutOff = params->others2.y;
    int framesToSkip = params->others2.z;
    int framesToWeight = params->others2.w;
    int currentFrameNo = (int)params->others3.x;
    float depthWeight = 1.0f;
    
    bool shortIteration = (iterationType == TRACKER_ITERATION_ROTATION) || (iterationType == TRACKER_ITERATION_TRANSLATION);
    
    int noPara = shortIteration ? 3 : 6, noParaSQ = shortIteration ? 6 : 21;
    
    float noValidPointsTotal = 0;
    localF = 0.0f;
    for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
    for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;
    
    for (int offY = 0; offY < pointsPerThread; offY++) for (int offX = 0; offX < pointsPerThread; offX++)
    {
        int locX = x * pointsPerThread + offX, locY = y * pointsPerThread + offY;
        int locId = locX + locY * params->viewImageSize.x;
        
        if (currentFrameNo < 100)
        {
            switch (iterationType)
            {
                case TRACKER_ITERATION_ROTATION:
                    isValidPoint = computePerPointGH_exDepth_Ab<true, true, false>(A, b, locX, locY, depth[locId], depthWeight,
                                                                                   params->viewImageSize, params->viewIntrinsics, params->sceneImageSize, params->sceneIntrinsics,
                                                                                   params->approxInvPose, params->scenePose, pointsMap, normalsMap, spaceThresh, viewFrustum_min, viewFrustum_max,
                                                                                   tukeyCutOff, framesToSkip, framesToWeight);
                    break;
                case TRACKER_ITERATION_TRANSLATION:
                    isValidPoint = computePerPointGH_exDepth_Ab<true, false, false>(A, b, locX, locY, depth[locId], depthWeight,
                                                                                    params->viewImageSize, params->viewIntrinsics, params->sceneImageSize, params->sceneIntrinsics,
                                                                                    params->approxInvPose, params->scenePose, pointsMap, normalsMap, spaceThresh, viewFrustum_min, viewFrustum_max,
                                                                                    tukeyCutOff, framesToSkip, framesToWeight);
                    break;
                case TRACKER_ITERATION_BOTH:
                    isValidPoint = computePerPointGH_exDepth_Ab<false, false, false>(A, b, locX, locY, depth[locId], depthWeight,
                                                                                     params->viewImageSize, params->viewIntrinsics, params->sceneImageSize, params->sceneIntrinsics,
                                                                                     params->approxInvPose, params->scenePose, pointsMap, normalsMap, spaceThresh, viewFrustum_min, viewFrustum_max,
                                                                                     tukeyCutOff, framesToSkip, framesToWeight);
                    break;
                default: break;
            }
        }
        else
        {
            switch (iterationType)
            {
                case TRACKER_ITERATION_ROTATION:
                    isValidPoint = computePerPointGH_exDepth_Ab<true, true, true>(A, b, locX, locY, depth[locId], depthWeight,
                                                                                  params->viewImageSize, params->viewIntrinsics, params->sceneImageSize, params->sceneIntrinsics,
                                                                                  params->approxInvPose, params->scenePose, pointsMap, normalsMap, spaceThresh, viewFrustum_min, viewFrustum_max,
                                                                                  tukeyCutOff, framesToSkip, framesToWeight);
                    break;
                case TRACKER_ITERATION_TRANSLATION:
                    isValidPoint = computePerPointGH_exDepth_Ab<true, false, true>(A, b, locX, locY, depth[locId], depthWeight,
                                                                                   params->viewImageSize, params->viewIntrinsics, params->sceneImageSize, params->sceneIntrinsics,
                                                                                   params->approxInvPose, params->scenePose, pointsMap, normalsMap, spaceThresh, viewFrustum_min, viewFrustum_max,
                                                                                   tukeyCutOff, framesToSkip, framesToWeight);
                    break;
                case TRACKER_ITERATION_BOTH:
                    isValidPoint = computePerPointGH_exDepth_Ab<false, false, true>(A, b, locX, locY, depth[locId], depthWeight,
                                                                                    params->viewImageSize, params->viewIntrinsics, params->sceneImageSize, params->sceneIntrinsics,
                                                                                    params->approxInvPose, params->scenePose, pointsMap, normalsMap, spaceThresh, viewFrustum_min, viewFrustum_max,
                                                                                    tukeyCutOff, framesToSkip, framesToWeight);
                    break;
                default: break;
            }
        }
        
        if (isValidPoint)
        {
            localF += rho(b, spaceThresh) * depthWeight;
            for (int i = 0; i < noPara; i++) localNabla[i] += rho_deriv(b, spaceThresh) * depthWeight * A[i];
        
            for (unsigned char r = 0, counter = 0; r < noPara; r++)
            {
                for (int c = 0; c <= r; c++, counter++) localHessian[counter] += rho_deriv2(b, spaceThresh) * depthWeight * A[r] * A[c];
            }
        
            noValidPointsTotal += (float)isValidPoint;
        }
    }
    
    if (noValidPointsTotal > 0)
    {
        int locId = x + y * (params->viewImageSize.x / pointsPerThread);
        
        noValidPoints[locId] = noValidPointsTotal;
        f[locId] = localF;
        for (int i = 0; i < noPara; i++) ATb[i + noPara * locId] = localNabla[i];
        for (int i = 0; i < noParaSQ; i++) ATA[i + noParaSQ * locId] = localHessian[i];
    }
}
