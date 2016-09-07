// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include <metal_stdlib>
using namespace metal;

#include "../Shared/ITMVisualisationEngine_Shared.h"
#include "ITMVisualisationEngine_Metal.h"
#include "../../../ITMLibDefines.h"

kernel void genericRaycastVH_device(DEVICEPTR(Vector4f) *pointsRay                                  [[ buffer(0) ]],
                                    DEVICEPTR(uchar) *entriesVisibleType                            [[ buffer(1) ]],
                                    const CONSTPTR(ITMVoxel) *voxelData                             [[ buffer(2) ]],
                                    const CONSTPTR(typename ITMVoxelIndex::IndexData) *voxelIndex   [[ buffer(3) ]],
                                    const CONSTPTR(Vector2f) *minmaxdata                            [[ buffer(4) ]],
                                    const CONSTPTR(CreateICPMaps_Params) *params                    [[ buffer(5) ]],
                                    uint2 threadIdx                                                 [[ thread_position_in_threadgroup ]],
                                    uint2 blockIdx                                                  [[ threadgroup_position_in_grid ]],
                                    uint2 blockDim                                                  [[ threads_per_threadgroup ]])
{
    int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;
    
    if (x >= params->imgSize.x || y >= params->imgSize.y) return;
    
    int locId = x + y * params->imgSize.x;
    int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * params->imgSize.x;
    
    if (params->imgSize.w > 0)
        castRay<ITMVoxel, ITMVoxelIndex, true>(pointsRay[locId], entriesVisibleType, x, y, voxelData, voxelIndex, params->invM, params->invProjParams,
                                               params->voxelSizes.y, params->lightSource.w, minmaxdata[locId2]);
    else
        castRay<ITMVoxel, ITMVoxelIndex, false>(pointsRay[locId], NULL, x, y, voxelData, voxelIndex, params->invM, params->invProjParams,
                                               params->voxelSizes.y, params->lightSource.w, minmaxdata[locId2]);
}

kernel void renderICP_device(const CONSTPTR(Vector4f) *pointsRay            [[ buffer(0) ]],
                             DEVICEPTR(Vector4f) *pointsMap                 [[ buffer(1) ]],
                             DEVICEPTR(Vector4f) *normalsMap                [[ buffer(2) ]],
                             DEVICEPTR(Vector4u) *outRendering              [[ buffer(3) ]],
                             const CONSTPTR(CreateICPMaps_Params) *params   [[ buffer(4) ]],
                             uint2 threadIdx                                [[ thread_position_in_threadgroup ]],
                             uint2 blockIdx                                 [[ threadgroup_position_in_grid ]],
                             uint2 blockDim                                 [[ threads_per_threadgroup ]])
{
    int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;
    
    if (x >= params->imgSize.x || y >= params->imgSize.y) return;
    
    processPixelICP<true, false>(pointsMap, normalsMap, pointsRay, params->imgSize.xy, x, y, params->voxelSizes.x, TO_VECTOR3(params->lightSource));
}