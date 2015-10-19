// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include <metal_stdlib>

#include "../../DeviceAgnostic/ITMDepthTracker.h"
#include "ITMDepthTracker_Metal.h"

using namespace metal;

kernel void depthTrackerOneLevel_g_rt_device(DEVICEPTR(float) *noValidPoints                            [[ buffer(0) ]],
                                             DEVICEPTR(float) *f                                        [[ buffer(1) ]],
                                             DEVICEPTR(float) *ATA                                      [[ buffer(2) ]],
                                             DEVICEPTR(float) *ATb                                      [[ buffer(3) ]],
                                             const CONSTPTR(float) *depth                              [[ buffer(4) ]],
                                             const CONSTPTR(Vector4f) *pointsMap                       [[ buffer(5) ]],
                                             const CONSTPTR(Vector4f) *normalsMap                      [[ buffer(6) ]],
                                             const CONSTPTR(DepthTrackerOneLevel_g_rg_Params) *params   [[ buffer(7) ]],
                                             uint2 threadIdx                                            [[ thread_position_in_threadgroup ]],
                                             uint2 blockIdx                                             [[ threadgroup_position_in_grid ]],
                                             uint2 blockDim                                             [[ threads_per_threadgroup ]])
{
    int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;
    
    float localNabla[6], localHessian[21], localF = 0; bool isValidPoint = false;
    
    int iterationType = (int)params->others[1];
    bool shortIteration = (iterationType == TRACKER_ITERATION_ROTATION) || (iterationType == TRACKER_ITERATION_TRANSLATION);
    
    int noPara = shortIteration ? 3 : 6, noParaSQ = shortIteration ? 6 : 21;
    
    for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
    for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;
    
    float noValidPointsTotal = 0;
    int ratio = (int)params->others.z;
    
    for (int offY = 0; offY < ratio; offY++) for (int offX = 0; offX < ratio; offX++)
    {
        int locX = x * ratio + offX, locY = y * ratio + offY;
        
        switch (iterationType)
        {
            case TRACKER_ITERATION_ROTATION:
                isValidPoint = computePerPointGH_Depth<true, true>(localNabla, localHessian, localF, locX, locY, depth[locX + locY * params->viewImageSize.x], params->viewImageSize,
                                                                   params->viewIntrinsics, params->sceneImageSize, params->sceneIntrinsics, params->approxInvPose,
                                                                   params->scenePose, pointsMap, normalsMap, params->others[0]);
                break;
            case TRACKER_ITERATION_TRANSLATION:
                isValidPoint = computePerPointGH_Depth<true, false>(localNabla, localHessian, localF, locX, locY, depth[locX + locY * params->viewImageSize.x], params->viewImageSize,
                                                                    params->viewIntrinsics, params->sceneImageSize, params->sceneIntrinsics, params->approxInvPose,
                                                                    params->scenePose, pointsMap, normalsMap, params->others[0]);
                break;
            case TRACKER_ITERATION_BOTH:
                isValidPoint = computePerPointGH_Depth<false, false>(localNabla, localHessian, localF, locX, locY, depth[locX + locY * params->viewImageSize.x], params->viewImageSize,
                                                                     params->viewIntrinsics, params->sceneImageSize, params->sceneIntrinsics, params->approxInvPose,
                                                                     params->scenePose, pointsMap, normalsMap, params->others[0]);
                break;
            default: break;
        }
        
        noValidPointsTotal += (float)isValidPoint;
    }
    
    if (noValidPointsTotal > 0)
    {
        int locId = x + y * (params->viewImageSize.x / ratio);
        
        noValidPoints[locId] = noValidPointsTotal;
        f[locId] = localF;
        for (int i = 0; i < noPara; i++) ATb[i + noPara * locId] = localNabla[i];
        for (int i = 0; i < noParaSQ; i++) ATA[i + noParaSQ * locId] = localHessian[i];
    }
}