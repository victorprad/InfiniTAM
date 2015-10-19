// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include <metal_stdlib>

#include "../../DeviceAgnostic/ITMViewBuilder.h"
#include "ITMViewBuilder_Metal.h"

using namespace metal;

kernel void convertDisparityToDepth_device(device float *d_out                                  [[ buffer(0) ]],
                                           const CONSTPTR(short) *d_in                          [[ buffer(1) ]],
                                           CONSTPTR(ConvertDisparityToDepth_Params*) params     [[ buffer(2) ]],
                                           uint2 threadIdx                                      [[ thread_position_in_threadgroup ]],
                                           uint2 blockIdx                                       [[ threadgroup_position_in_grid ]],
                                           uint2 blockDim                                       [[ threads_per_threadgroup ]])
{
    int x = threadIdx.x + blockIdx.x * blockDim.x;
    int y = threadIdx.y + blockIdx.y * blockDim.y;
    
    if ((x >= params->imgSize.x) || (y >= params->imgSize.y)) return;
    
    convertDisparityToDepth(d_out, x, y, d_in, params->disparityCalibParams, params->fx_depth, params->imgSize);
}