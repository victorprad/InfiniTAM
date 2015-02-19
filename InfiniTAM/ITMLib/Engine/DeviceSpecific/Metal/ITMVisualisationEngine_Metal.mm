// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#ifdef COMPILE_WITH_METAL

#import "MetalContext.h"

#include "ITMVisualisationEngine_Metal.h"
#include "../../DeviceAgnostic/ITMSceneReconstructionEngine.h"
#include "../../DeviceAgnostic/ITMVisualisationEngine.h"
#include "../../DeviceAgnostic/ITMSceneReconstructionEngine.h"
#include "../../../Objects/ITMRenderState_VH.h"

#include <vector>

id<MTLFunction> f_genericRaycastVH_device;
id<MTLComputePipelineState> p_genericRaycastVH_device;

id<MTLFunction> f_genericRaycastVGMissingPoints_device;
id<MTLComputePipelineState> p_genericRaycastVGMissingPoints_device;

id<MTLFunction> f_forwardProject_device;
id<MTLComputePipelineState> p_forwardProject_device;

id<MTLFunction> f_renderICP_device;
id<MTLComputePipelineState> p_renderICP_device;

id<MTLFunction> f_renderForward_device;
id<MTLComputePipelineState> p_renderForward_device;

id<MTLBuffer> paramsBuffer_visualisation;

using namespace ITMLib::Engine;

template<class TVoxel>
ITMVisualisationEngine_Metal<TVoxel, ITMVoxelBlockHash>::ITMVisualisationEngine_Metal(ITMScene<TVoxel, ITMVoxelBlockHash> *scene)
: ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash>(scene)
{
    NSError *errors;
    
    f_genericRaycastVH_device = [[[MetalContext instance]library]newFunctionWithName:@"genericRaycastVH_device"];
    p_genericRaycastVH_device = [[[MetalContext instance]device]newComputePipelineStateWithFunction:f_genericRaycastVH_device error:&errors];
    
    f_genericRaycastVGMissingPoints_device = [[[MetalContext instance]library]newFunctionWithName:@"genericRaycastVGMissingPoints_device"];
    p_genericRaycastVGMissingPoints_device = [[[MetalContext instance]device]newComputePipelineStateWithFunction:f_genericRaycastVGMissingPoints_device error:&errors];

    f_forwardProject_device = [[[MetalContext instance]library]newFunctionWithName:@"forwardProject_device"];
    p_forwardProject_device = [[[MetalContext instance]device]newComputePipelineStateWithFunction:f_forwardProject_device error:&errors];
    
    f_renderICP_device = [[[MetalContext instance]library]newFunctionWithName:@"renderICP_device"];
    p_renderICP_device = [[[MetalContext instance]device]newComputePipelineStateWithFunction:f_renderICP_device error:&errors];
    
    f_renderForward_device = [[[MetalContext instance]library]newFunctionWithName:@"renderForward_device"];
    p_renderForward_device = [[[MetalContext instance]device]newComputePipelineStateWithFunction:f_renderForward_device error:&errors];
    
    paramsBuffer_visualisation = BUFFEREMPTY(16384);
}

template<class TVoxel, class TIndex>
static void CreateICPMaps_common_metal(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState)
{
    id<MTLCommandBuffer> commandBuffer = [[[MetalContext instance]commandQueue]commandBuffer];
    id<MTLComputeCommandEncoder> commandEncoder = [commandBuffer computeCommandEncoder];
    
    trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
    
    CreateICPMaps_Params *params = (CreateICPMaps_Params*)[paramsBuffer_visualisation contents];
    params->imgSize.x = view->depth->noDims.x; params->imgSize.y = view->depth->noDims.y; params->imgSize.z = 0; params->imgSize.w = 0;
    params->voxelSizes.x = scene->sceneParams->voxelSize;
    params->voxelSizes.y = 1.0f / scene->sceneParams->voxelSize;
    params->invM = trackingState->pose_d->GetInvM();
    params->invProjParams = view->calib->intrinsics_d.projectionParamsSimple.all;
    params->invProjParams.x = 1.0f / params->invProjParams.x; params->invProjParams.y = 1.0f / params->invProjParams.y;
    params->lightSource.x = -Vector3f(params->invM.getColumn(2)).x;
    params->lightSource.y = -Vector3f(params->invM.getColumn(2)).y;
    params->lightSource.z = -Vector3f(params->invM.getColumn(2)).z;
    params->lightSource.w = scene->sceneParams->mu;
    
    [commandEncoder setComputePipelineState:p_genericRaycastVH_device];
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
    
    [commandEncoder setComputePipelineState:p_renderICP_device];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->raycastResult->GetMetalBuffer()             offset:0 atIndex:0];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) trackingState->pointCloud->locations->GetMetalBuffer()   offset:0 atIndex:1];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) trackingState->pointCloud->colours->GetMetalBuffer()     offset:0 atIndex:2];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->raycastImage->GetMetalBuffer()              offset:0 atIndex:3];
    [commandEncoder setBuffer:paramsBuffer_visualisation                                                        offset:0 atIndex:4];
    
    [commandEncoder dispatchThreadgroups:gridSize threadsPerThreadgroup:blockSize];
    [commandEncoder endEncoding];
    
    [commandBuffer commit];
    
    [commandBuffer waitUntilCompleted];
}

template<class TVoxel, class TIndex>
static void ForwardRender_common_metal(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState)
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
    
    CreateICPMaps_Params *params = (CreateICPMaps_Params*)[paramsBuffer_visualisation contents];
    params->imgSize.x = view->depth->noDims.x; params->imgSize.y = view->depth->noDims.y; params->imgSize.z = 0; params->imgSize.w = 0;
    params->voxelSizes.x = scene->sceneParams->voxelSize;
    params->voxelSizes.y = 1.0f / scene->sceneParams->voxelSize;
    params->M = trackingState->pose_d->GetM();
    params->invM = trackingState->pose_d->GetInvM();
    params->projParams = view->calib->intrinsics_d.projectionParamsSimple.all;
    params->invProjParams = view->calib->intrinsics_d.projectionParamsSimple.all;
    params->invProjParams.x = 1.0f / params->invProjParams.x;
    params->invProjParams.y = 1.0f / params->invProjParams.y;
    params->lightSource.x = -Vector3f(params->invM.getColumn(2)).x;
    params->lightSource.y = -Vector3f(params->invM.getColumn(2)).y;
    params->lightSource.z = -Vector3f(params->invM.getColumn(2)).z;
    params->lightSource.w = scene->sceneParams->mu;

    renderState->forwardProjection->Clear();
    
    MTLSize blockSize = {8, 8, 1};
    MTLSize gridSize = {(NSUInteger)ceil((float)view->depth->noDims.x / (float)blockSize.width),
        (NSUInteger)ceil((float)view->depth->noDims.y / (float)blockSize.height), 1};
    
    [commandEncoder setComputePipelineState:p_forwardProject_device];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->forwardProjection->GetMetalBuffer()     offset:0 atIndex:0];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->raycastResult->GetMetalBuffer()         offset:0 atIndex:1];
    [commandEncoder setBuffer:paramsBuffer_visualisation                                                    offset:0 atIndex:2];
    
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
        
        //if ((fwdPoint.w <= 0) && (minmaxval.x < minmaxval.y))
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
    
    [commandEncoder setComputePipelineState:p_genericRaycastVGMissingPoints_device];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->forwardProjection->GetMetalBuffer()         offset:0 atIndex:0];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->fwdProjMissingPoints->GetMetalBuffer()      offset:0 atIndex:1];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) scene->localVBA.GetVoxelBlocks_MB()                      offset:0 atIndex:2];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) scene->index.getIndexData_MB()                           offset:0 atIndex:3];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->renderingRangeImage->GetMetalBuffer()       offset:0 atIndex:4];
    [commandEncoder setBuffer:paramsBuffer_visualisation                                                        offset:0 atIndex:5];
    
    blockSize = {64, 1, 1};
    gridSize = {(NSUInteger)ceil((float)noMissingPoints / (float)blockSize.width), 1, 1};
    
    [commandEncoder dispatchThreadgroups:gridSize threadsPerThreadgroup:blockSize];
    [commandEncoder endEncoding];
    
    commandEncoder = [commandBuffer computeCommandEncoder];
    
    [commandEncoder setComputePipelineState:p_renderForward_device];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->raycastImage->GetMetalBuffer()              offset:0 atIndex:0];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState->forwardProjection->GetMetalBuffer()         offset:0 atIndex:1];
    [commandEncoder setBuffer:paramsBuffer_visualisation                                                        offset:0 atIndex:2];
    
    blockSize = {8, 8, 1};
    gridSize = {(NSUInteger)ceil((float)view->depth->noDims.x / (float)blockSize.width),
        (NSUInteger)ceil((float)view->depth->noDims.y / (float)blockSize.height), 1};
    
    [commandEncoder dispatchThreadgroups:gridSize threadsPerThreadgroup:blockSize];
    [commandEncoder endEncoding];
    
    [commandBuffer commit];
    
    [commandBuffer waitUntilCompleted];
}

template<class TVoxel>
void ITMVisualisationEngine_Metal<TVoxel, ITMVoxelBlockHash>::ForwardRender(const ITMView *view, ITMTrackingState *trackingState,
                                                                          ITMRenderState *renderState) const
{
    ForwardRender_common_metal(this->scene, view, trackingState, renderState);
}

template<class TVoxel>
void ITMVisualisationEngine_Metal<TVoxel, ITMVoxelBlockHash>::CreateICPMaps(const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const
{
    CreateICPMaps_common_metal(this->scene, view, trackingState, renderState);
}

template<class TVoxel>
void ITMVisualisationEngine_Metal<TVoxel,ITMVoxelBlockHash>::CreateExpectedDepths(const ITMPose *pose, const ITMIntrinsics *intrinsics,
                                                                                ITMRenderState *renderState) const
{
    Vector2i imgSize = renderState->renderingRangeImage->noDims;
    Vector2f *minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
    
    for (int locId = 0; locId < imgSize.x*imgSize.y; ++locId) {
        Vector2f & pixel = minmaxData[locId];
        pixel.x = FAR_AWAY;
        pixel.y = VERY_CLOSE;
    }

    float voxelSize = this->scene->sceneParams->voxelSize;
    
    std::vector<RenderingBlock> renderingBlocks(MAX_RENDERING_BLOCKS);
    int numRenderingBlocks = 0;
    
    ITMRenderState_VH* renderState_vh = (ITMRenderState_VH*)renderState;
    
    const int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();
    int noVisibleEntries = renderState_vh->noVisibleEntries;
    
    //go through list of visible 8x8x8 blocks
    for (int blockNo = 0; blockNo < noVisibleEntries; ++blockNo) {
        const ITMHashEntry & blockData(this->scene->index.GetEntries()[visibleEntryIDs[blockNo]]);
        
        Vector2i upperLeft, lowerRight;
        Vector2f zRange;
        bool validProjection = false;
        if (blockData.ptr>=0) {
            validProjection = ProjectSingleBlock(blockData.pos, pose->GetM(), intrinsics->projectionParamsSimple.all, imgSize, voxelSize, upperLeft, lowerRight, zRange);
        }
        if (!validProjection) continue;
        
        Vector2i requiredRenderingBlocks((int)ceilf((float)(lowerRight.x - upperLeft.x + 1) / (float)renderingBlockSizeX),
                                         (int)ceilf((float)(lowerRight.y - upperLeft.y + 1) / (float)renderingBlockSizeY));
        int requiredNumBlocks = requiredRenderingBlocks.x * requiredRenderingBlocks.y;
        
        if (numRenderingBlocks + requiredNumBlocks >= MAX_RENDERING_BLOCKS) continue;
        int offset = numRenderingBlocks;
        numRenderingBlocks += requiredNumBlocks;
        
        CreateRenderingBlocks(&(renderingBlocks[0]), offset, upperLeft, lowerRight, zRange);
    }
    
    // go through rendering blocks
    for (int blockNo = 0; blockNo < numRenderingBlocks; ++blockNo) {
        // fill minmaxData
        const RenderingBlock & b(renderingBlocks[blockNo]);
        
        for (int y = b.upperLeft.y; y <= b.lowerRight.y; ++y) {
            for (int x = b.upperLeft.x; x <= b.lowerRight.x; ++x) {
                Vector2f & pixel(minmaxData[x + y*imgSize.x]);
                if (pixel.x > b.zRange.x) pixel.x = b.zRange.x;
                if (pixel.y < b.zRange.y) pixel.y = b.zRange.y;
            }
        }
    }
}

template class ITMLib::Engine::ITMVisualisationEngine_Metal<ITMVoxel, ITMVoxelIndex>;

#endif