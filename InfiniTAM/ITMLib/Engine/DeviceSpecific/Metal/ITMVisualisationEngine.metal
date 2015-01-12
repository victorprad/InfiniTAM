// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include <metal_stdlib>

#include "../../DeviceAgnostic/ITMSceneReconstructionEngine.h"
#include "../../DeviceAgnostic/ITMVisualisationEngine.h"
#include "ITMVisualisationEngine_Metal.h"

using namespace metal;

kernel void createICPMaps_vh_device(DEVICEPTR(float) *depth                                        [[ buffer(0) ]],
                                    DEVICEPTR(Vector4f) *pointsMap                                 [[ buffer(1) ]],
                                    const DEVICEPTR(ITMVoxel) *voxelData                           [[ buffer(2) ]],
                                    const DEVICEPTR(typename ITMVoxelIndex::IndexData) *voxelIndex [[ buffer(3) ]],
                                    const DEVICEPTR(Vector2f) *minmaxdata                          [[ buffer(4) ]],
                                    const CONSTANT(CreateICPMaps_Params) *params                   [[ buffer(5) ]],
                                    uint2 threadIdx                                                [[ thread_position_in_threadgroup ]],
                                    uint2 blockIdx                                                 [[ threadgroup_position_in_grid ]],
                                    uint2 blockDim                                                 [[ threads_per_threadgroup ]])
{
//    int x = (threadIdx.x + blockIdx.x * blockDim.x) * 1, y = (threadIdx.y + blockIdx.y * blockDim.y) * 1;
//    
//    if (x >= params->imgSize.x || y >= params->imgSize.y) return;
//    
//    Vector3f pt_ray;
//    
//    int locId = x + y * params->imgSize.x;
//    
//    float viewFrustum_min = minmaxdata[locId].x;
//    float viewFrustum_max = minmaxdata[locId].y;
//    
//    bool foundPoint = false;
//    foundPoint = castRay<ITMVoxel,ITMVoxelIndex>(pt_ray, x, y, voxelData, voxelIndex, params->invM, params->projParams, params->imgSize,
//                                                 params->voxelSizes.y, params->lightSource.w, viewFrustum_min, viewFrustum_max);
//    
//    if (foundPoint) pointsMap[locId] = Vector4f(pt_ray.x, pt_ray.y, pt_ray.z, 1.0f);
    
//    if (foundPoint)
//    {
//        Vector3f outNormal;
//        float angle = 0.0f;
//        outNormal = normalize(computeSingleNormalFromSDF<ITMVoxel, ITMVoxelIndex>(voxelData, voxelIndex, pt_ray));
//        
//        angle = dot(outNormal, params->lightSource.xyz);
//        if (!(angle > 0.0f)) return;
//        
//        uchar imageValue = (uchar)((0.8f * angle + 0.2f) * 255.0f);
//        
//        outRendering[x + y * params->imgSize.x] = imageValue;
//        
//        Vector4f outNormal4; outNormal4.xyz = outNormal; outNormal4.w = 0.0f;
//        Vector4f outPoint4; outPoint4.xyz = pt_ray * params->voxelSizes.x; outPoint4.w = 1.0f;
//        
//        pointsMap[locId] = outPoint4; normalsMap[locId] = outNormal4;
//        
////        locId = (x + 0) + (y + 1) * params->imgSize.x;
////        pointsMap[locId] = outPoint4; normalsMap[locId] = outNormal4; outRendering[locId] = imageValue;
////        
////        locId = (x + 1) + (y + 1) * params->imgSize.x;
////        pointsMap[locId] = outPoint4; normalsMap[locId] = outNormal4; outRendering[locId] = imageValue;
////
////        locId = (x + 1) + (y + 0) * params->imgSize.x;
////        pointsMap[locId] = outPoint4; normalsMap[locId] = outNormal4; outRendering[locId] = imageValue;
//    }
}

kernel void createICPMaps_vh_normals_device(DEVICEPTR(Vector4f) *pointsMap                                 [[ buffer(0) ]],
                                            DEVICEPTR(Vector4f) *normalsMap                                [[ buffer(1) ]],
                                            DEVICEPTR(Vector4u) *outRendering                              [[ buffer(2) ]],
                                            const DEVICEPTR(ITMVoxel) *voxelData                           [[ buffer(3) ]],
                                            const DEVICEPTR(typename ITMVoxelIndex::IndexData) *voxelIndex [[ buffer(4) ]],
                                            const CONSTANT(CreateICPMaps_Params) *params                   [[ buffer(5) ]],
                                            uint2 threadIdx                                                [[ thread_position_in_threadgroup ]],
                                            uint2 blockIdx                                                 [[ threadgroup_position_in_grid ]],
                                            uint2 blockDim                                                 [[ threads_per_threadgroup ]])
{
//    int x = (threadIdx.x + blockIdx.x * blockDim.x) * 1, y = (threadIdx.y + blockIdx.y * blockDim.y) * 1;
//    
//    if (x >= params->imgSize.x || y >= params->imgSize.y) return;
//    
//    int locId = x + y * params->imgSize.x;
//    
//    Vector4f pt_ray = pointsMap[locId];
//    if (pt_ray.w > 0)
//    {
//        Vector3f outNormal;
//        float angle = 0.0f;
//        
//        outNormal = normalize(computeSingleNormalFromSDF<ITMVoxel, ITMVoxelIndex>(voxelData, voxelIndex, pt_ray.xyz));
//        
//        angle = dot(outNormal, params->lightSource.xyz);
//        if (!(angle > 0.0f)) return;
//        
//        uchar imageValue = (uchar)((0.8f * angle + 0.2f) * 255.0f);
//        
//        outRendering[x + y * params->imgSize.x] = imageValue;
//        
//        Vector4f outNormal4; outNormal4.xyz = outNormal; outNormal4.w = 0.0f;
//        Vector4f outPoint4; outPoint4.xyz = pt_ray.xyz * params->voxelSizes.x; outPoint4.w = 1.0f;
//        
//        pointsMap[locId] = outPoint4; normalsMap[locId] = outNormal4;
//        
////        locId = (x + 0) + (y + 1) * params->imgSize.x;
////        pointsMap[locId] = outPoint4; normalsMap[locId] = outNormal4; outRendering[locId] = imageValue;
////        
////        locId = (x + 1) + (y + 1) * params->imgSize.x;
////        pointsMap[locId] = outPoint4; normalsMap[locId] = outNormal4; outRendering[locId] = imageValue;
////        
////        locId = (x + 1) + (y + 0) * params->imgSize.x;
////        pointsMap[locId] = outPoint4; normalsMap[locId] = outNormal4; outRendering[locId] = imageValue;
//    }
}