// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include <metal_stdlib>

#include "../../DeviceAgnostic/ITMDepthTracker.h"
#include "ITMDepthTracker_Metal.h"

using namespace metal;

kernel void depthTrackerOneLevel_g_rt_device(DEVICEPTR(float) *noValidPoints                            [[ buffer(0) ]],
                                             DEVICEPTR(float) *ATA                                      [[ buffer(1) ]],
                                             DEVICEPTR(float) *ATb                                      [[ buffer(2) ]],
                                             const DEVICEPTR(float) *depth                              [[ buffer(3) ]],
                                             const DEVICEPTR(Vector4f) *pointsMap                       [[ buffer(4) ]],
                                             const DEVICEPTR(Vector4f) *normalsMap                      [[ buffer(5) ]],
                                             const CONSTANT(DepthTrackerOneLevel_g_rg_Params) *params   [[ buffer(6) ]],
                                             uint2 threadIdx                                            [[ thread_position_in_threadgroup ]],
                                             uint2 blockIdx                                             [[ threadgroup_position_in_grid ]],
                                             uint2 blockDim                                             [[ threads_per_threadgroup ]])
{
    int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;
    
    float localNabla[6], localHessian[21]; bool isValidPoint = false;
    
    bool rotationOnly = (bool)params->others[1];
    
    int noPara = rotationOnly ? 3 : 6, noParaSQ = rotationOnly ? 6 : 21;
    
    if (x < params->viewImageSize.x && y < params->viewImageSize.y)
    {
        int locId = x + y * params->viewImageSize.x;
        
        if (rotationOnly)
            isValidPoint = computePerPointGH_Depth<true>(localNabla, localHessian, x, y, depth[locId], params->viewImageSize,
                                                         params->viewIntrinsics, params->sceneImageSize, params->sceneIntrinsics, params->approxInvPose,
                                                         params->scenePose, pointsMap, normalsMap, params->others[0]);
        else
            isValidPoint = computePerPointGH_Depth<false>(localNabla, localHessian, x, y, depth[locId], params->viewImageSize,
                                                          params->viewIntrinsics, params->sceneImageSize, params->sceneIntrinsics, params->approxInvPose,
                                                          params->scenePose, pointsMap, normalsMap, params->others[0]);

        if (isValidPoint)
        {
            noValidPoints[locId] = isValidPoint;
            for (int i = 0; i < noPara; i++) ATb[i + noPara * locId] = localNabla[i];
            for (int i = 0; i < noParaSQ; i++) ATA[i + noParaSQ * locId] = localHessian[i];
        }
    }
}