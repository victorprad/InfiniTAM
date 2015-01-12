// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM
#ifdef COMPILE_WITH_METAL

#import "ITMMetalContext.h"

#include "ITMSceneReconstructionEngine_Metal.h"
#include "../../DeviceAgnostic/ITMSceneReconstructionEngine.h"

id<MTLFunction> f_integrateIntoScene_vh_device;
id<MTLComputePipelineState> p_integrateIntoScene_vh_device;

id<MTLBuffer> paramsBuffer_sceneReconstruction;

using namespace ITMLib::Engine;

template<class TVoxel>
ITMSceneReconstructionEngine_Metal<TVoxel,ITMVoxelBlockHash>::ITMSceneReconstructionEngine_Metal(void)
{
    int noTotalEntries = ITMVoxelBlockHash::noVoxelBlocks;
    entriesAllocType = (uchar*)malloc(noTotalEntries);
    blockCoords = (Vector3s*)malloc(noTotalEntries * sizeof(Vector3s));

    NSError *errors;
    f_integrateIntoScene_vh_device = [[[ITMMetalContext instance]library]newFunctionWithName:@"integrateIntoScene_vh_device"];
    p_integrateIntoScene_vh_device = [[[ITMMetalContext instance]device]newComputePipelineStateWithFunction:f_integrateIntoScene_vh_device error:&errors];
    
    paramsBuffer_sceneReconstruction = BUFFEREMPTY(16384);
}

template<class TVoxel>
ITMSceneReconstructionEngine_Metal<TVoxel,ITMVoxelBlockHash>::~ITMSceneReconstructionEngine_Metal(void)
{
    free(entriesAllocType);
    free(blockCoords);
}

template<class TVoxel>
void ITMSceneReconstructionEngine_Metal<TVoxel,ITMVoxelBlockHash>::IntegrateIntoScene(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, const ITMPose *pose_d)
{
    id<MTLCommandBuffer> commandBuffer = [[[ITMMetalContext instance]commandQueue]commandBuffer];
    id<MTLComputeCommandEncoder> commandEncoder = [commandBuffer computeCommandEncoder];
    
    IntegrateIntoScene_VH_Params *params = (IntegrateIntoScene_VH_Params*)[paramsBuffer_sceneReconstruction contents];
    params->rgbImgSize = view->rgb->noDims;
    params->depthImgSize = view->depth->noDims;
    params->_voxelSize = scene->sceneParams->voxelSize;
    params->M_d = pose_d->M;
    if (TVoxel::hasColorInformation) params->M_rgb = view->calib->trafo_rgb_to_depth.calib_inv * pose_d->M;
    
    params->projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
    params->projParams_rgb = view->calib->intrinsics_rgb.projectionParamsSimple.all;
    
    params->mu = scene->sceneParams->mu; params->maxW = scene->sceneParams->maxW;
    
    [commandEncoder setComputePipelineState:p_integrateIntoScene_vh_device];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) scene->localVBA.GetVoxelBlocks_MB()  offset:0 atIndex:0];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) scene->index.GetEntries_MB()         offset:0 atIndex:1];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) scene->index.GetLiveEntryIDs_MB()    offset:0 atIndex:2];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) view->rgb->GetMetalBuffer()          offset:0 atIndex:3];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) view->depth->GetMetalBuffer()        offset:0 atIndex:4];
    [commandEncoder setBuffer:paramsBuffer_sceneReconstruction                              offset:0 atIndex:5];
    
    MTLSize blockSize = {SDF_BLOCK_SIZE, SDF_BLOCK_SIZE, SDF_BLOCK_SIZE};
    MTLSize gridSize = {(NSUInteger)scene->index.noLiveEntries, 1, 1};
    
    [commandEncoder dispatchThreadgroups:gridSize threadsPerThreadgroup:blockSize];
    [commandEncoder endEncoding];
    
    [commandBuffer commit];
    
    [commandBuffer waitUntilCompleted];
}

template<class TVoxel>
void ITMSceneReconstructionEngine_Metal<TVoxel,ITMVoxelBlockHash>::AllocateSceneFromDepth(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, const ITMPose *pose_d)
{
    Vector2i depthImgSize = view->depth->noDims;
    float voxelSize = scene->sceneParams->voxelSize;
    
    Matrix4f M_d, invM_d;
    Vector4f projParams_d, invProjParams_d;
    
    M_d = pose_d->M; M_d.inv(invM_d);
    
    projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
    invProjParams_d = projParams_d;
    invProjParams_d.x = 1.0f / invProjParams_d.x;
    invProjParams_d.y = 1.0f / invProjParams_d.y;
    
    float mu = scene->sceneParams->mu;
    
    float *depth = view->depth->GetData(false);
    int *voxelAllocationList = scene->localVBA.GetAllocationList();
    int *excessAllocationList = scene->index.GetExcessAllocationList();
    uchar *entriesVisibleType = scene->index.GetEntriesVisibleType();
    ITMHashEntry *hashTable = scene->index.GetEntries();
    ITMHashCacheState *cacheStates = scene->useSwapping ? scene->globalCache->GetCacheStates(false) : 0;
    int *liveEntryIDs = scene->index.GetLiveEntryIDs();
    int noTotalEntries = scene->index.noVoxelBlocks;
    
    bool useSwapping = scene->useSwapping;
    
    float oneOverVoxelSize = 1.0f / (voxelSize * SDF_BLOCK_SIZE);
    
    int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
    int lastFreeExcessListId = scene->index.lastFreeExcessListId;
    
    Vector3s pt_block_prev;
    pt_block_prev.x = 0; pt_block_prev.y = 0; pt_block_prev.z = 0;
    
    int hashIdxLive = 0;
    
    memset(entriesAllocType, 0, noTotalEntries);
    memset(entriesVisibleType, 0, noTotalEntries);
    memset(blockCoords, 0, noTotalEntries * sizeof(Vector3s));
    
    //build hashVisibility
    for (int y = 0; y < depthImgSize.y; y++) for (int x = 0; x < depthImgSize.x; x++)
    {
        buildHashAllocAndVisibleTypePP(entriesAllocType, entriesVisibleType, x, y, blockCoords, depth, invM_d,
                                       invProjParams_d, mu, depthImgSize, oneOverVoxelSize, hashTable, scene->sceneParams->viewFrustum_min,
                                       scene->sceneParams->viewFrustum_max);
    }
    
    //allocate
    for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
    {
        int vbaIdx, exlIdx;
        unsigned char hashChangeType = entriesAllocType[targetIdx];
        ITMHashEntry hashEntry = hashTable[targetIdx];
        
        switch (hashChangeType)
        {
            case 1: //needs allocation, fits in the ordered list
                vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;
                
                if (vbaIdx >= 0) //there is room in the voxel block array
                {
                    Vector3s pt_block_all = blockCoords[targetIdx];
                    
                    hashEntry.pos.x = pt_block_all.x; hashEntry.pos.y = pt_block_all.y; hashEntry.pos.z = pt_block_all.z;
                    hashEntry.ptr = voxelAllocationList[vbaIdx];
                    
                    hashTable[targetIdx] = hashEntry;
                }
                
                break;
            case 2: //needs allocation in the excess list
                vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;
                exlIdx = lastFreeExcessListId; lastFreeExcessListId--;
                
                if (vbaIdx >= 0 && exlIdx >= 0) //there is room in the voxel block array and excess list
                {
                    Vector3s pt_block_all = blockCoords[targetIdx];
                    
                    hashEntry.pos.x = pt_block_all.x; hashEntry.pos.y = pt_block_all.y; hashEntry.pos.z = pt_block_all.z;
                    hashEntry.ptr = voxelAllocationList[vbaIdx];
                    
                    int exlOffset = excessAllocationList[exlIdx];
                    
                    hashTable[targetIdx].offset = exlOffset + 1; //connect to child
                    
                    hashTable[SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + exlOffset] = hashEntry; //add child to the excess list
                    
                    entriesVisibleType[SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + exlOffset] = 1; //make child visible
                }
                
                break;
        }
    }
    
    //build visible list
    for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
    {
        unsigned char hashVisibleType = entriesVisibleType[targetIdx];
        const ITMHashEntry &hashEntry = hashTable[targetIdx];
        
        if ((hashVisibleType == 0 && hashEntry.ptr >= 0) || hashVisibleType == 2)
        {
            Vector3f pt_image, buff3f;
            
            int noInvisible = 0, noInvisibleEnlarged = 0;
            
            pt_image = hashEntry.pos.toFloat() * (float)SDF_BLOCK_SIZE * voxelSize;
            buff3f = M_d * pt_image;
            
            if (buff3f.z > 1e-10f)
            {
                for (int x = 0; x <= 1; x++) for (int y = 0; y <= 1; y++) for (int z = 0; z <= 1; z++)
                {
                    Vector3f off((float)x, (float)y, (float)z);
                    
                    pt_image = (hashEntry.pos.toFloat() + off) * (float)SDF_BLOCK_SIZE * voxelSize;
                    
                    buff3f = M_d * pt_image;
                    
                    pt_image.x = projParams_d.x * buff3f.x / buff3f.z + projParams_d.z;
                    pt_image.y = projParams_d.y * buff3f.y / buff3f.z + projParams_d.w;
                    
                    if (!(pt_image.x >= 0 && pt_image.x < depthImgSize.x && pt_image.y >= 0 && pt_image.y < depthImgSize.y)) noInvisible++;
                    
                    if (useSwapping)
                    {
                        Vector4i lims;
                        lims.x = -depthImgSize.x / 8; lims.y = depthImgSize.x + depthImgSize.x / 8;
                        lims.z = -depthImgSize.y / 8; lims.w = depthImgSize.y + depthImgSize.y / 8;
                        
                        if (!(pt_image.x >= lims.x && pt_image.x < lims.y && pt_image.y >= lims.z && pt_image.y < lims.w)) noInvisibleEnlarged++;
                    }
                }
                
                hashVisibleType = noInvisible < 8;
                
                if (useSwapping) entriesVisibleType[targetIdx] = noInvisibleEnlarged < 8;
            }
        }
        
        if (useSwapping)
        {
            if (entriesVisibleType[targetIdx] > 0 && cacheStates[targetIdx].cacheFromHost != 2) cacheStates[targetIdx].cacheFromHost = 1;
        }
        
        if (hashVisibleType > 0)
        {
            liveEntryIDs[hashIdxLive] = targetIdx;
            hashIdxLive++;
        }
    }
    
    //reallocate deletes ones from previous swap operation
    if (useSwapping)
    {
        for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
        {
            int vbaIdx;
            ITMHashEntry hashEntry = hashTable[targetIdx];
            
            if (entriesVisibleType[targetIdx] > 0 && hashEntry.ptr == -1)
            {
                vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;
                if (vbaIdx >= 0) hashTable[targetIdx].ptr = voxelAllocationList[vbaIdx];
            }
        }
    }
    
    scene->index.noLiveEntries = hashIdxLive;
    scene->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
    scene->index.lastFreeExcessListId = lastFreeExcessListId;
}

template<class TVoxel>
ITMSceneReconstructionEngine_Metal<TVoxel,ITMPlainVoxelArray>::ITMSceneReconstructionEngine_Metal(void)
{}

template<class TVoxel>
ITMSceneReconstructionEngine_Metal<TVoxel,ITMPlainVoxelArray>::~ITMSceneReconstructionEngine_Metal(void)
{}

template<class TVoxel>
void ITMSceneReconstructionEngine_Metal<TVoxel,ITMPlainVoxelArray>::AllocateSceneFromDepth(ITMScene<TVoxel,ITMPlainVoxelArray> *scene, const ITMView *view, const ITMPose *pose_d)
{}

template<class TVoxel>
void ITMSceneReconstructionEngine_Metal<TVoxel,ITMPlainVoxelArray>::IntegrateIntoScene(ITMScene<TVoxel,ITMPlainVoxelArray> *scene, const ITMView *view, const ITMPose *pose_d)
{
    Vector2i rgbImgSize = view->rgb->noDims;
    Vector2i depthImgSize = view->depth->noDims;
    float voxelSize = scene->sceneParams->voxelSize;
    
    Matrix4f M_d, M_rgb;
    Vector4f projParams_d, projParams_rgb;
    
    M_d = pose_d->M;
    if (TVoxel::hasColorInformation) M_rgb = view->calib->trafo_rgb_to_depth.calib_inv * pose_d->M;
    
    projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
    projParams_rgb = view->calib->intrinsics_rgb.projectionParamsSimple.all;
    
    float mu = scene->sceneParams->mu; int maxW = scene->sceneParams->maxW;
    
    float *depth = view->depth->GetData(false);
    Vector4u *rgb = view->rgb->GetData(false);
    TVoxel *voxelArray = scene->localVBA.GetVoxelBlocks();
    
    const ITMPlainVoxelArray::IndexData *arrayInfo = scene->index.getIndexData();
    
    for (int z = 0; z < scene->index.getVolumeSize().z; z++) for (int y = 0; y < scene->index.getVolumeSize().y; y++) for (int x = 0; x < scene->index.getVolumeSize().x; x++)
    {
        Vector4f pt_model; int locId;
        
        locId = x + y * scene->index.getVolumeSize().x + z * scene->index.getVolumeSize().x * scene->index.getVolumeSize().y;
        
        pt_model.x = (float)(x + arrayInfo->offset.x) * voxelSize;
        pt_model.y = (float)(y + arrayInfo->offset.y) * voxelSize;
        pt_model.z = (float)(z + arrayInfo->offset.z) * voxelSize;
        pt_model.w = 1.0f;
        
        ComputeUpdatedVoxelInfo<TVoxel::hasColorInformation,TVoxel>::compute(voxelArray[locId], pt_model, M_d, projParams_d, M_rgb, projParams_rgb, mu, maxW, depth, depthImgSize, rgb, rgbImgSize);
    }
}

template class ITMLib::Engine::ITMSceneReconstructionEngine_Metal<ITMVoxel, ITMVoxelIndex>;


#endif