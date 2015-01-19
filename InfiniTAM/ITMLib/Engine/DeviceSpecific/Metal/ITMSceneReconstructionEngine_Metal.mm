// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#ifdef COMPILE_WITH_METAL

#import "MetalContext.h"

#include "../../../Objects/ITMRenderState_VH.h"

#include "ITMSceneReconstructionEngine_Metal.h"
#include "../../DeviceAgnostic/ITMSceneReconstructionEngine.h"

id<MTLFunction> f_integrateIntoScene_vh_device;
id<MTLComputePipelineState> p_integrateIntoScene_vh_device;

id<MTLBuffer> paramsBuffer_sceneReconstruction;

using namespace ITMLib::Engine;

template<class TVoxel>
ITMSceneReconstructionEngine_Metal<TVoxel,ITMVoxelBlockHash>::ITMSceneReconstructionEngine_Metal(void)
 : ITMSceneReconstructionEngine_CPU<TVoxel,ITMVoxelBlockHash>()
{
    NSError *errors;
    f_integrateIntoScene_vh_device = [[[MetalContext instance]library]newFunctionWithName:@"integrateIntoScene_vh_device"];
    p_integrateIntoScene_vh_device = [[[MetalContext instance]device]newComputePipelineStateWithFunction:f_integrateIntoScene_vh_device error:&errors];
    
    paramsBuffer_sceneReconstruction = BUFFEREMPTY(16384);
}

template<class TVoxel>
void ITMSceneReconstructionEngine_Metal<TVoxel,ITMVoxelBlockHash>::IntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
                                                                                      const ITMTrackingState *trackingState, const ITMRenderState *renderState)
{
    id<MTLCommandBuffer> commandBuffer = [[[MetalContext instance]commandQueue]commandBuffer];
    id<MTLComputeCommandEncoder> commandEncoder = [commandBuffer computeCommandEncoder];
    
    ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState;
    
    IntegrateIntoScene_VH_Params *params = (IntegrateIntoScene_VH_Params*)[paramsBuffer_sceneReconstruction contents];
    params->rgbImgSize = view->rgb->noDims;
    params->depthImgSize = view->depth->noDims;
    params->_voxelSize = scene->sceneParams->voxelSize;
    params->M_d = trackingState->pose_d->M;
    if (TVoxel::hasColorInformation) params->M_rgb = view->calib->trafo_rgb_to_depth.calib_inv * trackingState->pose_d->M;
    
    params->projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
    params->projParams_rgb = view->calib->intrinsics_rgb.projectionParamsSimple.all;
    
    params->mu = scene->sceneParams->mu; params->maxW = scene->sceneParams->maxW;
    
    [commandEncoder setComputePipelineState:p_integrateIntoScene_vh_device];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) scene->localVBA.GetVoxelBlocks_MB()      offset:0 atIndex:0];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) scene->index.GetEntries_MB()             offset:0 atIndex:1];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState_vh->GetLiveEntryIDs_MB()     offset:0 atIndex:2];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) view->rgb->GetMetalBuffer()              offset:0 atIndex:3];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) view->depth->GetMetalBuffer()            offset:0 atIndex:4];
    [commandEncoder setBuffer:paramsBuffer_sceneReconstruction                                  offset:0 atIndex:5];
    
    MTLSize blockSize = {SDF_BLOCK_SIZE, SDF_BLOCK_SIZE, SDF_BLOCK_SIZE};
    MTLSize gridSize = {(NSUInteger)renderState_vh->noLiveEntries, 1, 1};
    
    [commandEncoder dispatchThreadgroups:gridSize threadsPerThreadgroup:blockSize];
    [commandEncoder endEncoding];
    
    [commandBuffer commit];
    
    [commandBuffer waitUntilCompleted];
}

template class ITMLib::Engine::ITMSceneReconstructionEngine_Metal<ITMVoxel, ITMVoxelIndex>;

#endif