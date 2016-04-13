// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include <metal_stdlib>

using namespace metal;

#include "../Shared/ITMSceneReconstructionEngine_Shared.h"
#include "ITMSceneReconstructionEngine_Metal.h"
#include "../../../ITMLibDefines.h"

kernel void integrateIntoScene_vh_device(DEVICEPTR(ITMVoxel) *localVBA                          [[ buffer(0) ]],
                                         const CONSTPTR(ITMHashEntry) *hashTable                [[ buffer(1) ]],
                                         DEVICEPTR(int) *visibleEntryIDs                        [[ buffer(2) ]],
                                         const CONSTPTR(Vector4u) *rgb                          [[ buffer(3) ]],
                                         const CONSTPTR(float) *depth                           [[ buffer(4) ]],
                                         const CONSTPTR(IntegrateIntoScene_VH_Params) *params   [[ buffer(5) ]],
                                         uint3 threadIdx                                        [[ thread_position_in_threadgroup ]],
                                         uint3 blockIdx                                         [[ threadgroup_position_in_grid ]],
                                         uint3 blockDim                                         [[ threads_per_threadgroup ]])
{
    Vector3i globalPos;
    int entryId = visibleEntryIDs[blockIdx.x];

    const CONSTPTR(ITMHashEntry) &currentHashEntry = hashTable[entryId];

    if (currentHashEntry.ptr < 0) return;

    globalPos = (int3)currentHashEntry.pos * SDF_BLOCK_SIZE;

    DEVICEPTR(ITMVoxel) *localVoxelBlock = &(localVBA[currentHashEntry.ptr * SDF_BLOCK_SIZE3]);
    
    int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;

    Vector4f pt_model; int locId;

    locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
    
    pt_model.x = (float)(globalPos.x + x) * params->others.x;
    pt_model.y = (float)(globalPos.y + y) * params->others.x;
    pt_model.z = (float)(globalPos.z + z) * params->others.x;
    pt_model.w = 1.0f;
    
    ComputeUpdatedVoxelInfo<ITMVoxel::hasColorInformation,false, ITMVoxel>::compute(localVoxelBlock[locId], pt_model,
                                                                         params->M_d, params->projParams_d, params->M_rgb,
                                                                         params->projParams_rgb, params->others.y, params->others.z, depth, NULL,
                                                                         params->depthImgSize, rgb, params->rgbImgSize);
}

kernel void buildAllocAndVisibleType_vh_device(DEVICEPTR(unsigned char) *entriesAllocType                   [[ buffer(0) ]],
                                               DEVICEPTR(unsigned char) *entriesVisibleType                 [[ buffer(1) ]],
                                               DEVICEPTR(Vector4s) *blockCoords                             [[ buffer(2) ]],
                                               const CONSTPTR(ITMHashEntry) *hashTable                      [[ buffer(3) ]],
                                               const CONSTPTR(float) *depth                                 [[ buffer(4) ]],
                                               const CONSTPTR(BuildAllocVisibleType_VH_Params) *params      [[ buffer(5) ]],
                                               uint3 threadIdx                                              [[ thread_position_in_threadgroup ]],
                                               uint3 blockIdx                                               [[ threadgroup_position_in_grid ]],
                                               uint3 blockDim                                               [[ threads_per_threadgroup ]])
{
    int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;
    
    if (x >= params->depthImgSize.x || y >= params->depthImgSize.y) return;
    
    buildHashAllocAndVisibleTypePP(entriesAllocType, entriesVisibleType, x, y, blockCoords, depth, params->invM_d,
                                   params->invProjParams_d, params->others.x, params->depthImgSize, params->others.y,
                                   hashTable, params->others.z, params->others.w);
}
