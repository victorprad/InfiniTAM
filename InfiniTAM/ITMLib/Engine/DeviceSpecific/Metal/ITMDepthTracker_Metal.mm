// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDepthTracker_Metal.h"
#include "../../DeviceAgnostic/ITMDepthTracker.h"
#import <Accelerate/Accelerate.h>

using namespace ITMLib::Engine;

id<MTLFunction> f_depthTrackerOneLevel_g_rg_device;
id<MTLComputePipelineState> p_depthTrackerOneLevel_g_rg_device;

id<MTLBuffer> paramsBuffer_depthTracker;

ITMDepthTracker_Metal::ITMDepthTracker_Metal(Vector2i imgSize, int noHierarchyLevels, int noRotationOnlyLevels, int noICPRunTillLevel, float distThresh, ITMLowLevelEngine *lowLevelEngine)
:ITMDepthTracker(imgSize, noHierarchyLevels, noRotationOnlyLevels, noICPRunTillLevel, distThresh, lowLevelEngine, MEMORYDEVICE_CPU)
{
    allocImgSize = imgSize;

    allocateMetalData((void**)&ATb_metal, (void**)&ATb_metal_mb, allocImgSize.x * allocImgSize.y * 6 * sizeof(float), true);
    allocateMetalData((void**)&ATA_metal, (void**)&ATA_metal_mb, allocImgSize.x * allocImgSize.y * 21 * sizeof(float), true);
    allocateMetalData((void**)&noValidPoints_metal, (void**)&noValidPoints_metal_mb, allocImgSize.x * allocImgSize.y * sizeof(float), true);

    NSError *errors;
    f_depthTrackerOneLevel_g_rg_device = [[[MetalContext instance]library]newFunctionWithName:@"depthTrackerOneLevel_g_rt_device"];
    p_depthTrackerOneLevel_g_rg_device = [[[MetalContext instance]device]newComputePipelineStateWithFunction:f_depthTrackerOneLevel_g_rg_device error:&errors];

    paramsBuffer_depthTracker = BUFFEREMPTY(16384);
}

ITMDepthTracker_Metal::~ITMDepthTracker_Metal(void)
{
    freeMetalData((void**)&ATb_metal, (void**)&ATb_metal_mb, allocImgSize.x * allocImgSize.y * 6 * sizeof(float), true);
    freeMetalData((void**)&ATA_metal, (void**)&ATA_metal_mb, allocImgSize.x * allocImgSize.y * 21 * sizeof(float), true);
    freeMetalData((void**)&noValidPoints_metal, (void**)&noValidPoints_metal_mb, allocImgSize.x * allocImgSize.y * sizeof(float), true);
}

int ITMDepthTracker_Metal::ComputeGandH(ITMSceneHierarchyLevel *sceneHierarchyLevel, ITMTemplatedHierarchyLevel<ITMFloatImage> *viewHierarchyLevel,
                                      Matrix4f approxInvPose, Matrix4f scenePose, bool rotationOnly)
{
    int noValidPoints;
    
    Vector4f sceneIntrinsics = sceneHierarchyLevel->intrinsics;
    Vector2i sceneImageSize = sceneHierarchyLevel->pointsMap->noDims;
    Vector4f viewIntrinsics = viewHierarchyLevel->intrinsics;
    Vector2i viewImageSize = viewHierarchyLevel->depth->noDims;
    
    float packedATA[6 * 6];
    int noPara = rotationOnly ? 3 : 6, noParaSQ = rotationOnly ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;
    
    int viewImageTotalSize = viewImageSize.x * viewImageSize.y;
    
    noValidPoints = 0; memset(ATA_host, 0, sizeof(float) * 6 * 6); memset(ATb_host, 0, sizeof(float) * 6);
    memset(packedATA, 0, sizeof(float) * noParaSQ);
    
    id<MTLCommandBuffer> commandBuffer = [[[MetalContext instance]commandQueue]commandBuffer];
    id<MTLComputeCommandEncoder> commandEncoder = [commandBuffer computeCommandEncoder];

//    memset(noValidPoints_metal, 0, allocImgSize.x * allocImgSize.y * sizeof(float));
//    memset(ATb_metal, 0, allocImgSize.x * allocImgSize.y * 6 * sizeof(float));
//    memset(ATA_metal, 0, allocImgSize.x * allocImgSize.y * 21 * sizeof(float));
    
    DepthTrackerOneLevel_g_rg_Params *params = (DepthTrackerOneLevel_g_rg_Params*)[paramsBuffer_depthTracker contents];
    params->approxInvPose = approxInvPose; params->sceneIntrinsics = sceneIntrinsics;
    params->sceneImageSize = sceneImageSize; params->scenePose = scenePose;
    params->viewIntrinsics = viewIntrinsics; params->viewImageSize = viewImageSize;
    params->others.x = distThresh; params->others.y = (float)rotationOnly;
    
    [commandEncoder setComputePipelineState:p_depthTrackerOneLevel_g_rg_device];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) noValidPoints_metal_mb                               offset:0 atIndex:0];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) ATA_metal_mb                                         offset:0 atIndex:1];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) ATb_metal_mb                                         offset:0 atIndex:2];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) viewHierarchyLevel->depth->GetMetalBuffer()          offset:0 atIndex:3];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) sceneHierarchyLevel->pointsMap->GetMetalBuffer()     offset:0 atIndex:4];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) sceneHierarchyLevel->normalsMap->GetMetalBuffer()    offset:0 atIndex:5];
    [commandEncoder setBuffer:paramsBuffer_depthTracker                                                     offset:0 atIndex:6];

    MTLSize blockSize = {16, 16, 1};
    MTLSize gridSize = {(NSUInteger)ceil((float)viewImageSize.x / (float)blockSize.width),
        (NSUInteger)ceil((float)viewImageSize.y / (float)blockSize.height), 1};
    
    memset(noValidPoints_metal, 0, sizeof(float) * viewImageTotalSize);
    
    [commandEncoder dispatchThreadgroups:gridSize threadsPerThreadgroup:blockSize];
    [commandEncoder endEncoding];

    [commandBuffer commit];

    [commandBuffer waitUntilCompleted];
   
    for (int locId = 0; locId < viewImageTotalSize; locId++)
    {
        if (noValidPoints_metal[locId])
        {
            noValidPoints++;
            for (int i = 0; i < noPara; i++) ATb_host[i] += ATb_metal[i + noPara * locId];
            for (int i = 0; i < noParaSQ; i++) packedATA[i] += ATA_metal[i + noParaSQ * locId];
        }
    }
    
    for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++) ATA_host[r + c * 6] = packedATA[counter];
    for (int r = 0; r < noPara; ++r) for (int c = r + 1; c < noPara; c++) ATA_host[r + c * 6] = ATA_host[c + r * 6];
    
    return noValidPoints;
}
