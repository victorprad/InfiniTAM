// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#ifdef COMPILE_WITH_METAL

#import "MetalContext.h"

#include "ITMVisualisationEngine_Metal.h"
#include "../Shared/ITMVisualisationEngine_Shared.h"
#include "../../../Objects/RenderStates/ITMRenderState_VH.h"

#include <vector>

using namespace ITMLib;

template<class TVoxel>
ITMVisualisationEngine_Metal<TVoxel, ITMVoxelBlockHash>::ITMVisualisationEngine_Metal()
: ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash>()
{
    NSError *errors;
    
    metalBits.f_genericRaycastVH_device = [[[MetalContext instance]library]newFunctionWithName:@"genericRaycastVH_device"];
    metalBits.p_genericRaycastVH_device = [[[MetalContext instance]device]newComputePipelineStateWithFunction:metalBits.f_genericRaycastVH_device error:&errors];
    
    metalBits.f_genericRaycastVHMissingPoints_device = [[[MetalContext instance]library]newFunctionWithName:@"genericRaycastVHMissingPoints_device"];
    metalBits.p_genericRaycastVHMissingPoints_device = [[[MetalContext instance]device]newComputePipelineStateWithFunction:metalBits.f_genericRaycastVHMissingPoints_device error:&errors];

    metalBits.f_forwardProject_device = [[[MetalContext instance]library]newFunctionWithName:@"forwardProject_device"];
    metalBits.p_forwardProject_device = [[[MetalContext instance]device]newComputePipelineStateWithFunction:metalBits.f_forwardProject_device error:&errors];
    
    metalBits.f_renderICP_device = [[[MetalContext instance]library]newFunctionWithName:@"renderICP_device"];
    metalBits.p_renderICP_device = [[[MetalContext instance]device]newComputePipelineStateWithFunction:metalBits.f_renderICP_device error:&errors];
    
    metalBits.f_renderForward_device = [[[MetalContext instance]library]newFunctionWithName:@"renderForward_device"];
    metalBits.p_renderForward_device = [[[MetalContext instance]device]newComputePipelineStateWithFunction:metalBits.f_renderForward_device error:&errors];
    
    metalBits.paramsBuffer_visualisation = BUFFEREMPTY(16384);
}

template<class TVoxel, class TIndex>
static void CreateICPMaps_common_metal(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState,
                                       const VisualisationEngine_MetalBits *metalBits)
{
    const void *entriesVisibleType = NULL;
    if ((dynamic_cast<const ITMRenderState_VH*>(renderState)!=NULL))
    {
        entriesVisibleType = ((ITMRenderState_VH*)renderState)->GetEntriesVisibleType_MB();
    }

    id<MTLCommandBuffer> commandBuffer = [[[MetalContext instance]commandQueue]commandBuffer];
    id<MTLComputeCommandEncoder> commandEncoder = [commandBuffer computeCommandEncoder];
    
    trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
    
    CreateICPMaps_Params *params = (CreateICPMaps_Params*)[metalBits->paramsBuffer_visualisation contents];
    params->imgSize.x = view->depth->noDims.x; params->imgSize.y = view->depth->noDims.y; params->imgSize.z = 0; params->imgSize.w = 0;
    params->voxelSizes.x = scene->sceneParams->voxelSize;
    params->voxelSizes.y = 1.0f / scene->sceneParams->voxelSize;
    params->invM = trackingState->pose_d->GetInvM();
    params->invProjParams = InvertProjectionParams(view->calib->intrinsics_d.projectionParamsSimple.all);
    params->lightSource.x = -Vector3f(params->invM.getColumn(2)).x;
    params->lightSource.y = -Vector3f(params->invM.getColumn(2)).y;
    params->lightSource.z = -Vector3f(params->invM.getColumn(2)).z;
    params->lightSource.w = scene->sceneParams->mu;
    
    [commandEncoder setComputePipelineState:metalBits->p_genericRaycastVH_device];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->raycastResult->GetMetalBuffer()             offset:0 atIndex:0];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) entriesVisibleType                                       offset:0 atIndex:1];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) scene->localVBA.GetVoxelBlocks_MB()                      offset:0 atIndex:2];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) scene->index.getIndexData_MB()                           offset:0 atIndex:3];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->renderingRangeImage->GetMetalBuffer()       offset:0 atIndex:4];
    [commandEncoder setBuffer:metalBits->paramsBuffer_visualisation                                             offset:0 atIndex:5];
    
    MTLSize blockSize = {16, 16, 1};
    MTLSize gridSize = {(NSUInteger)ceil((float)view->depth->noDims.x / (float)blockSize.width),
        (NSUInteger)ceil((float)view->depth->noDims.y / (float)blockSize.height), 1};
    
    [commandEncoder dispatchThreadgroups:gridSize threadsPerThreadgroup:blockSize];
    [commandEncoder endEncoding];
    
    commandEncoder = [commandBuffer computeCommandEncoder];
    
    [commandEncoder setComputePipelineState:metalBits->p_renderICP_device];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->raycastResult->GetMetalBuffer()             offset:0 atIndex:0];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) trackingState->pointCloud->locations->GetMetalBuffer()   offset:0 atIndex:1];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) trackingState->pointCloud->colours->GetMetalBuffer()     offset:0 atIndex:2];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->raycastImage->GetMetalBuffer()              offset:0 atIndex:3];
    [commandEncoder setBuffer:metalBits->paramsBuffer_visualisation                                             offset:0 atIndex:4];
    
    [commandEncoder dispatchThreadgroups:gridSize threadsPerThreadgroup:blockSize];
    [commandEncoder endEncoding];
    
    [commandBuffer commit];
    
    [commandBuffer waitUntilCompleted];
}

template<class TVoxel, class TIndex>
static void ForwardRender_common_metal(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState,
                                       const VisualisationEngine_MetalBits *metalBits)
{  
    Vector2i imgSize = view->depth->noDims;
    Matrix4f M = trackingState->pose_d->GetM();
    Matrix4f invM = trackingState->pose_d->GetInvM();
    Vector4f projParams = view->calib->intrinsics_d.projectionParamsSimple.all;
    
    Vector3f lightSource = -Vector3f(invM.getColumn(2));
    const Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
    Vector4f *forwardProjection = renderState->forwardProjection->GetData(MEMORYDEVICE_CPU);
    float *currentDepth = view->depth->GetData(MEMORYDEVICE_CPU);
    int *fwdProjMissingPoints = renderState->fwdProjMissingPoints->GetData(MEMORYDEVICE_CPU);
    Vector4u *outRendering = renderState->raycastImage->GetData(MEMORYDEVICE_CPU);
    const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
    float voxelSize = scene->sceneParams->voxelSize;
    const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
    const typename TIndex::IndexData *voxelIndex = scene->index.getIndexData();
    
    id<MTLCommandBuffer> commandBuffer = [[[MetalContext instance]commandQueue]commandBuffer];
    id<MTLComputeCommandEncoder> commandEncoder = [commandBuffer computeCommandEncoder];
    
    CreateICPMaps_Params *params = (CreateICPMaps_Params*)[metalBits->paramsBuffer_visualisation contents];
    params->imgSize.x = view->depth->noDims.x; params->imgSize.y = view->depth->noDims.y; params->imgSize.z = 0; params->imgSize.w = 0;
    params->voxelSizes.x = scene->sceneParams->voxelSize;
    params->voxelSizes.y = 1.0f / scene->sceneParams->voxelSize;
    params->M = trackingState->pose_d->GetM();
    params->invM = trackingState->pose_d->GetInvM();
    params->projParams = view->calib->intrinsics_d.projectionParamsSimple.all;
    params->invProjParams = InvertProjectionParams(view->calib->intrinsics_d.projectionParamsSimple.all);
    params->lightSource.x = -Vector3f(params->invM.getColumn(2)).x;
    params->lightSource.y = -Vector3f(params->invM.getColumn(2)).y;
    params->lightSource.z = -Vector3f(params->invM.getColumn(2)).z;
    params->lightSource.w = scene->sceneParams->mu;

    renderState->forwardProjection->Clear();
    
    MTLSize blockSize = {16, 16, 1};
    MTLSize gridSize = {(NSUInteger)ceil((float)view->depth->noDims.x / (float)blockSize.width),
        (NSUInteger)ceil((float)view->depth->noDims.y / (float)blockSize.height), 1};
    
    [commandEncoder setComputePipelineState:metalBits->p_forwardProject_device];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->forwardProjection->GetMetalBuffer()     offset:0 atIndex:0];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->raycastResult->GetMetalBuffer()         offset:0 atIndex:1];
    [commandEncoder setBuffer:metalBits->paramsBuffer_visualisation                                         offset:0 atIndex:2];
    
    [commandEncoder dispatchThreadgroups:gridSize threadsPerThreadgroup:blockSize];
    [commandEncoder endEncoding];
    
    [commandBuffer commit];
    
    [commandBuffer waitUntilCompleted];
    
    int noMissingPoints = 0;
    for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
    {
        int locId = x + y * imgSize.x;
        int locId2 = (x / minmaximg_subsample) + (y / minmaximg_subsample) * imgSize.x;
        
        Vector4f fwdPoint = forwardProjection[locId];
        Vector2f minmaxval = minmaximg[locId2];
        float depth = currentDepth[locId];
        
        if ((fwdPoint.w <= 0) && ((fwdPoint.x == 0 && fwdPoint.y == 0 && fwdPoint.z == 0) || (depth >= 0)) && (minmaxval.x < minmaxval.y))
        {
            fwdProjMissingPoints[noMissingPoints] = locId;
            noMissingPoints++;
        }
    }
    
    renderState->noFwdProjMissingPoints = noMissingPoints;
    
    params->imgSize.x = view->depth->noDims.x; params->imgSize.y = view->depth->noDims.y; params->imgSize.z = noMissingPoints; params->imgSize.w = 0;
    
    commandBuffer = [[[MetalContext instance]commandQueue]commandBuffer];
    commandEncoder = [commandBuffer computeCommandEncoder];
    
    [commandEncoder setComputePipelineState:metalBits->p_genericRaycastVHMissingPoints_device];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->forwardProjection->GetMetalBuffer()         offset:0 atIndex:0];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->fwdProjMissingPoints->GetMetalBuffer()      offset:0 atIndex:1];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) scene->localVBA.GetVoxelBlocks_MB()                      offset:0 atIndex:2];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) scene->index.getIndexData_MB()                           offset:0 atIndex:3];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->renderingRangeImage->GetMetalBuffer()       offset:0 atIndex:4];
    [commandEncoder setBuffer:metalBits->paramsBuffer_visualisation                                             offset:0 atIndex:5];
    
    blockSize = {64, 1, 1};
    gridSize = {(NSUInteger)ceil((float)noMissingPoints / (float)blockSize.width), 1, 1};
    
    [commandEncoder dispatchThreadgroups:gridSize threadsPerThreadgroup:blockSize];
    [commandEncoder endEncoding];
    
    commandEncoder = [commandBuffer computeCommandEncoder];
    
    [commandEncoder setComputePipelineState:metalBits->p_renderForward_device];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->raycastImage->GetMetalBuffer()              offset:0 atIndex:0];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->forwardProjection->GetMetalBuffer()         offset:0 atIndex:1];
    [commandEncoder setBuffer:metalBits->paramsBuffer_visualisation                                             offset:0 atIndex:2];
    
    blockSize = {8, 8, 1};
    gridSize = {(NSUInteger)ceil((float)view->depth->noDims.x / (float)blockSize.width),
        (NSUInteger)ceil((float)view->depth->noDims.y / (float)blockSize.height), 1};
    
    [commandEncoder dispatchThreadgroups:gridSize threadsPerThreadgroup:blockSize];
    [commandEncoder endEncoding];
    
    [commandBuffer commit];
    
    [commandBuffer waitUntilCompleted];
}

template<class TVoxel>
void ITMVisualisationEngine_Metal<TVoxel, ITMVoxelBlockHash>::ForwardRender(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const
{
    ForwardRender_common_metal(scene, view, trackingState, renderState, &metalBits);
}

template<class TVoxel>
void ITMVisualisationEngine_Metal<TVoxel, ITMVoxelBlockHash>::CreateICPMaps(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const
{
    CreateICPMaps_common_metal(scene, view, trackingState, renderState, &metalBits);
}

#endif
