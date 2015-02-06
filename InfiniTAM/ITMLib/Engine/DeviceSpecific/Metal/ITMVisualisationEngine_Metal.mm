// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#ifdef COMPILE_WITH_METAL

#import "MetalContext.h"

#include "ITMVisualisationEngine_Metal.h"
#include "../../DeviceAgnostic/ITMSceneReconstructionEngine.h"
#include "../../DeviceAgnostic/ITMVisualisationEngine.h"
#include "../../DeviceAgnostic/ITMSceneReconstructionEngine.h"
#include "../../../Objects/ITMRenderState_VH.h"

id<MTLFunction> f_createICPMaps_vh_device;
id<MTLComputePipelineState> p_createICPMaps_vh_device;

id<MTLFunction> f_createICPMaps_vh_normals_device;
id<MTLComputePipelineState> p_createICPMaps_vh_normals_device;

id<MTLBuffer> paramsBuffer_visualisation;

using namespace ITMLib::Engine;

template<class TVoxel>
ITMVisualisationEngine_Metal<TVoxel, ITMVoxelBlockHash>::ITMVisualisationEngine_Metal(ITMScene<TVoxel, ITMVoxelBlockHash> *scene)
: ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash>(scene)
{
    NSError *errors;
    
    f_createICPMaps_vh_device = [[[MetalContext instance]library]newFunctionWithName:@"createICPMaps_vh_device"];
    p_createICPMaps_vh_device = [[[MetalContext instance]device]newComputePipelineStateWithFunction:f_createICPMaps_vh_device error:&errors];
    
    f_createICPMaps_vh_normals_device = [[[MetalContext instance]library]newFunctionWithName:@"createICPMaps_vh_normals_device"];
    p_createICPMaps_vh_normals_device = [[[MetalContext instance]device]newComputePipelineStateWithFunction:f_createICPMaps_vh_normals_device error:&errors];
    
    paramsBuffer_visualisation = BUFFEREMPTY(16384);
}

template<class TVoxel>
void ITMVisualisationEngine_Metal<TVoxel, ITMVoxelBlockHash>::CreateICPMaps(const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const
{
    CreateICPMaps_common_metal(this->scene, view, trackingState, renderState);
}

template<class TVoxel, class TIndex>
static void CreateICPMaps_common_metal(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState)
{
    id<MTLCommandBuffer> commandBuffer = [[[MetalContext instance]commandQueue]commandBuffer];
    id<MTLComputeCommandEncoder> commandEncoder = [commandBuffer computeCommandEncoder];
    
    trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
    
    CreateICPMaps_Params *params = (CreateICPMaps_Params*)[paramsBuffer_visualisation contents];
    params->imgSize = view->depth->noDims;
    params->voxelSizes.x = scene->sceneParams->voxelSize;
    params->voxelSizes.y = 1.0f / scene->sceneParams->voxelSize;
    params->invM = trackingState->pose_d->GetInvM();
    params->projParams = view->calib->intrinsics_d.projectionParamsSimple.all;
    params->projParams.x = 1.0f / params->projParams.x; params->projParams.y = 1.0f / params->projParams.y;
    params->lightSource.x = -Vector3f(params->invM.getColumn(2)).x;
    params->lightSource.y = -Vector3f(params->invM.getColumn(2)).y;
    params->lightSource.z = -Vector3f(params->invM.getColumn(2)).z;
    params->lightSource.w = scene->sceneParams->mu;
    
    [commandEncoder setComputePipelineState:p_createICPMaps_vh_device];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->raycastResult->GetMetalBuffer()             offset:0 atIndex:0];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) scene->localVBA.GetVoxelBlocks_MB()                      offset:0 atIndex:1];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) scene->index.getIndexData_MB()                           offset:0 atIndex:2];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->renderingRangeImage->GetMetalBuffer()       offset:0 atIndex:3];
    [commandEncoder setBuffer:paramsBuffer_visualisation                                                        offset:0 atIndex:4];
    
    MTLSize blockSize = {8, 8, 1};
    MTLSize gridSize = {(NSUInteger)ceil((float)view->depth->noDims.x / (float)blockSize.width),
        (NSUInteger)ceil((float)view->depth->noDims.y / (float)blockSize.height), 1};
    
    [commandEncoder dispatchThreadgroups:gridSize threadsPerThreadgroup:blockSize];
    [commandEncoder endEncoding];
    
    commandEncoder = [commandBuffer computeCommandEncoder];
    
    [commandEncoder setComputePipelineState:p_createICPMaps_vh_normals_device];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->raycastResult->GetMetalBuffer()             offset:0 atIndex:0];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) trackingState->pointCloud->locations->GetMetalBuffer()   offset:0 atIndex:1];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) trackingState->pointCloud->colours->GetMetalBuffer()     offset:0 atIndex:2];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->raycastImage->GetMetalBuffer()              offset:0 atIndex:3];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) scene->localVBA.GetVoxelBlocks_MB()                      offset:0 atIndex:4];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) scene->index.getIndexData_MB()                           offset:0 atIndex:5];
    [commandEncoder setBuffer:paramsBuffer_visualisation                                                        offset:0 atIndex:6];
    
    [commandEncoder dispatchThreadgroups:gridSize threadsPerThreadgroup:blockSize];
    [commandEncoder endEncoding];
    
    [commandBuffer commit];
    
    [commandBuffer waitUntilCompleted];
}

template class ITMLib::Engine::ITMVisualisationEngine_Metal<ITMVoxel, ITMVoxelIndex>;

#endif