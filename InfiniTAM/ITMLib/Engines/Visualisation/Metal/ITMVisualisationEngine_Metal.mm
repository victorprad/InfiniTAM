// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#ifdef COMPILE_WITH_METAL

#import "MetalContext.h"

#include "ITMVisualisationEngine_Metal.h"
#include "../Shared/ITMVisualisationEngine_Shared.h"
#include "../../../Objects/RenderStates/ITMRenderState_VH.h"

#include <vector>

using namespace ITMLib;

struct VisualisationEngine_MetalBits
{
    id<MTLFunction> f_genericRaycastVH_device;
    id<MTLComputePipelineState> p_genericRaycastVH_device;
    
    id<MTLFunction> f_renderICP_device;
    id<MTLComputePipelineState> p_renderICP_device;
    
    id<MTLBuffer> paramsBuffer;
};

static VisualisationEngine_MetalBits vis_metalBits;

template<class TVoxel>
ITMVisualisationEngine_Metal<TVoxel, ITMVoxelBlockHash>::ITMVisualisationEngine_Metal()
: ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash>()
{
    NSError *errors;
    
    vis_metalBits.f_genericRaycastVH_device = [[[MetalContext instance]library]newFunctionWithName:@"genericRaycastVH_device"];
    vis_metalBits.p_genericRaycastVH_device = [[[MetalContext instance]device]newComputePipelineStateWithFunction:vis_metalBits.f_genericRaycastVH_device error:&errors];
    
    vis_metalBits.f_renderICP_device = [[[MetalContext instance]library]newFunctionWithName:@"renderICP_device"];
    vis_metalBits.p_renderICP_device = [[[MetalContext instance]device]newComputePipelineStateWithFunction:vis_metalBits.f_renderICP_device error:&errors];
    
    vis_metalBits.paramsBuffer = BUFFEREMPTY(16384);
}

template<class TVoxel, class TIndex>
static void CreateICPMaps_common_metal(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState)
{
    const void *entriesVisibleType = NULL;
    if ((dynamic_cast<const ITMRenderState_VH*>(renderState)!=NULL))
    {
        entriesVisibleType = ((ITMRenderState_VH*)renderState)->GetEntriesVisibleType_MB();
    }

    id<MTLCommandBuffer> commandBuffer = [[[MetalContext instance]commandQueue]commandBuffer];
    id<MTLComputeCommandEncoder> commandEncoder = [commandBuffer computeCommandEncoder];
    
    trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
    
    CreateICPMaps_Params *params = (CreateICPMaps_Params*)[vis_metalBits.paramsBuffer contents];
    params->imgSize.x = view->depth->noDims.x; params->imgSize.y = view->depth->noDims.y; params->imgSize.z = 0; params->imgSize.w = 0;
    params->voxelSizes.x = scene->sceneParams->voxelSize;
    params->voxelSizes.y = 1.0f / scene->sceneParams->voxelSize;
    params->invM = trackingState->pose_d->GetInvM();
    params->invProjParams = InvertProjectionParams(view->calib->intrinsics_d.projectionParamsSimple.all);
    params->lightSource.x = -Vector3f(params->invM.getColumn(2)).x;
    params->lightSource.y = -Vector3f(params->invM.getColumn(2)).y;
    params->lightSource.z = -Vector3f(params->invM.getColumn(2)).z;
    params->lightSource.w = scene->sceneParams->mu;
    
    [commandEncoder setComputePipelineState:vis_metalBits.p_genericRaycastVH_device];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->raycastResult->GetMetalBuffer()             offset:0 atIndex:0];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) entriesVisibleType                                       offset:0 atIndex:1];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) scene->localVBA.GetVoxelBlocks_MB()                      offset:0 atIndex:2];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) scene->index.getIndexData_MB()                           offset:0 atIndex:3];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->renderingRangeImage->GetMetalBuffer()       offset:0 atIndex:4];
    [commandEncoder setBuffer:vis_metalBits.paramsBuffer                                                        offset:0 atIndex:5];
    
    MTLSize blockSize = {16, 16, 1};
    MTLSize gridSize = {(NSUInteger)ceil((float)view->depth->noDims.x / (float)blockSize.width),
        (NSUInteger)ceil((float)view->depth->noDims.y / (float)blockSize.height), 1};
    
    [commandEncoder dispatchThreadgroups:gridSize threadsPerThreadgroup:blockSize];
    [commandEncoder endEncoding];
    
    commandEncoder = [commandBuffer computeCommandEncoder];
    
    [commandEncoder setComputePipelineState:vis_metalBits.p_renderICP_device];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->raycastResult->GetMetalBuffer()             offset:0 atIndex:0];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) trackingState->pointCloud->locations->GetMetalBuffer()   offset:0 atIndex:1];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) trackingState->pointCloud->colours->GetMetalBuffer()     offset:0 atIndex:2];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->raycastImage->GetMetalBuffer()              offset:0 atIndex:3];
    [commandEncoder setBuffer:vis_metalBits.paramsBuffer                                                        offset:0 atIndex:4];
    
    [commandEncoder dispatchThreadgroups:gridSize threadsPerThreadgroup:blockSize];
    [commandEncoder endEncoding];
    
    [commandBuffer commit];
    
    [commandBuffer waitUntilCompleted];
}

template<class TVoxel>
void ITMVisualisationEngine_Metal<TVoxel, ITMVoxelBlockHash>::CreateICPMaps(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const
{
    CreateICPMaps_common_metal(scene, view, trackingState, renderState);
}

#endif
