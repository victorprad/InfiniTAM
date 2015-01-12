// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include <metal_stdlib>

#include "../../DeviceAgnostic/ITMDepthTracker.h"
#include "ITMDepthTracker_Metal.h"

using namespace metal;

kernel void depthTrackerOneLevel_g_rt_device(DEVICEPTR(float) *noValidPoints                            [[ buffer(0) ]],
                                             DEVICEPTR(float) *ATA                                      [[ buffer(1) ]],
                                             DEVICEPTR(float) *ATb                                      [[ buffer(2) ]],
                                             CONSTANT(float) *depth                                    [[ buffer(3) ]],
                                             CONSTANT(Vector4f) *pointsMap                             [[ buffer(4) ]],
                                             CONSTANT(Vector4f) *normalsMap                            [[ buffer(5) ]],
                                             const CONSTANT(DepthTrackerOneLevel_g_rg_Params) *params   [[ buffer(6) ]],
                                             uint2 threadIdx                                            [[ thread_position_in_threadgroup ]],
                                             uint2 blockIdx                                             [[ threadgroup_position_in_grid ]],
                                             uint2 blockDim                                             [[ threads_per_threadgroup ]])
{
//    int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;
//    
//    float localNabla[6], localHessian[21]; bool isValidPoint = false;
//    
//    int noPara = (bool)params->others[1] ? 3 : 6, noParaSQ = (bool)params->others[1] ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;
//    
//    if (x >= 0 && x < params->viewImageSize.x && y >= 0 && y < params->viewImageSize.y)
//    {
//        isValidPoint = computePerPointGH_Depth(localNabla, localHessian, x, y, depth, params->viewImageSize, params->viewIntrinsics, params->sceneImageSize, params->sceneIntrinsics,
//                                               params->approxInvPose, params->scenePose, pointsMap, normalsMap, params->others[0], (bool)params->others[1], noPara);
//
//        if (isValidPoint)
//        {
//            int locId = x + y * params->viewImageSize.x;
//            
//            noValidPoints[locId] = isValidPoint;
//            for (int i = 0; i < noPara; i++) ATb[i + noPara * locId] = localNabla[i];
//            for (int i = 0; i < noParaSQ; i++) ATA[i + noParaSQ * locId] = localHessian[i];
//        }
//    }
}