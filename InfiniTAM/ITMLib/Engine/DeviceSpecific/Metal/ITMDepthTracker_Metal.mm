// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDepthTracker_Metal.h"
#include "../../DeviceAgnostic/ITMDepthTracker.h"
#import <Accelerate/Accelerate.h>

using namespace ITMLib::Engine;

id<MTLFunction> f_depthTrackerOneLevel_g_rg_device;
id<MTLComputePipelineState> p_depthTrackerOneLevel_g_rg_device;

id<MTLBuffer> paramsBuffer_depthTracker;

ITMDepthTracker_Metal::ITMDepthTracker_Metal(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
                                             int noICPRunTillLevel, float distThresh, const ITMLowLevelEngine *lowLevelEngine)
:ITMDepthTracker(imgSize, trackingRegime, noHierarchyLevels, noICPRunTillLevel, distThresh, lowLevelEngine, MEMORYDEVICE_CPU)
{
    allocImgSize = imgSize;

    allocateMetalData((void**)&ATb_metal, (void**)&ATb_metal_mb, allocImgSize.x * allocImgSize.y * 6 * sizeof(float), true);
    allocateMetalData((void**)&ATA_metal, (void**)&ATA_metal_mb, allocImgSize.x * allocImgSize.y * 21 * sizeof(float), true);
    allocateMetalData((void**)&noValidPoints_metal, (void**)&noValidPoints_metal_mb, allocImgSize.x * allocImgSize.y * sizeof(float), true);
    allocateMetalData((void**)&f_metal, (void**)&f_metal_mb, allocImgSize.x * allocImgSize.y * sizeof(float), true);
    
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
    freeMetalData((void**)&f_metal, (void**)&f_metal_mb, allocImgSize.x * allocImgSize.y * sizeof(float), true);
}

int ITMDepthTracker_Metal::ComputeGandH(ITMSceneHierarchyLevel *sceneHierarchyLevel, ITMTemplatedHierarchyLevel<ITMFloatImage> *viewHierarchyLevel,
                                        Matrix4f approxInvPose, Matrix4f scenePose, TrackerIterationType iterationType)
{
    int noValidPoints;
    
    Vector4f sceneIntrinsics = sceneHierarchyLevel->intrinsics;
    Vector2i sceneImageSize = sceneHierarchyLevel->pointsMap->noDims;
    Vector4f viewIntrinsics = viewHierarchyLevel->intrinsics;
    Vector2i viewImageSize = viewHierarchyLevel->depth->noDims;
    
    noValidPoints = 0; memset(ATA_host, 0, sizeof(float) * 6 * 6); memset(ATb_host, 0, sizeof(float) * 6); f = 0;
    if (iterationType == TRACKER_ITERATION_NONE) return 0;
    
    bool shortIteration = (iterationType == TRACKER_ITERATION_ROTATION) || (iterationType == TRACKER_ITERATION_TRANSLATION);
    
    float packedATA[6 * 6];
    int noPara = shortIteration ? 3 : 6, noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;
    
    int viewImageTotalSize = viewImageSize.x * viewImageSize.y;
    int ratio = 4;
    
    memset(packedATA, 0, sizeof(float) * noParaSQ);
    
    id<MTLCommandBuffer> commandBuffer = [[[MetalContext instance]commandQueue]commandBuffer];
    id<MTLComputeCommandEncoder> commandEncoder = [commandBuffer computeCommandEncoder];
    
    DepthTrackerOneLevel_g_rg_Params *params = (DepthTrackerOneLevel_g_rg_Params*)[paramsBuffer_depthTracker contents];
    params->approxInvPose = approxInvPose; params->sceneIntrinsics = sceneIntrinsics;
    params->sceneImageSize = sceneImageSize; params->scenePose = scenePose;
    params->viewIntrinsics = viewIntrinsics; params->viewImageSize = viewImageSize;
    params->others.x = distThresh; params->others.y = (float)iterationType; params->others.z = (float)ratio; params->others.w = 0;
    
    [commandEncoder setComputePipelineState:p_depthTrackerOneLevel_g_rg_device];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) noValidPoints_metal_mb                               offset:0 atIndex:0];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) f_metal_mb                                           offset:0 atIndex:1];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) ATA_metal_mb                                         offset:0 atIndex:2];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) ATb_metal_mb                                         offset:0 atIndex:3];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) viewHierarchyLevel->depth->GetMetalBuffer()          offset:0 atIndex:4];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) sceneHierarchyLevel->pointsMap->GetMetalBuffer()     offset:0 atIndex:5];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) sceneHierarchyLevel->normalsMap->GetMetalBuffer()    offset:0 atIndex:6];
    [commandEncoder setBuffer:paramsBuffer_depthTracker                                                     offset:0 atIndex:7];
    
    MTLSize blockSize = {4, 4, 1};
    MTLSize gridSize = {(NSUInteger)ceil(((float)viewImageSize.x / ratio) / (float)blockSize.width),
        (NSUInteger)ceil(((float)viewImageSize.y / ratio) / (float)blockSize.height), 1};
    
    memset(noValidPoints_metal, 0, sizeof(float) * viewImageTotalSize);
    memset(f_metal, 0, sizeof(float) * viewImageTotalSize);
    
    [commandEncoder dispatchThreadgroups:gridSize threadsPerThreadgroup:blockSize];
    [commandEncoder endEncoding];

    [commandBuffer commit];

    [commandBuffer waitUntilCompleted];
   
    for (int locId = 0; locId < (viewImageTotalSize / (ratio * ratio)); locId++)
    {
        if (noValidPoints_metal[locId] > 0)
        {
            noValidPoints += noValidPoints_metal[locId];
            f += f_metal[locId];
            for (int i = 0; i < noPara; i++) ATb_host[i] += ATb_metal[i + noPara * locId];
            for (int i = 0; i < noParaSQ; i++) packedATA[i] += ATA_metal[i + noParaSQ * locId];
        }
    }
    
    for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++) ATA_host[r + c * 6] = packedATA[counter];
    for (int r = 0; r < noPara; ++r) for (int c = r + 1; c < noPara; c++) ATA_host[r + c * 6] = ATA_host[c + r * 6];
    
    return noValidPoints;
}
