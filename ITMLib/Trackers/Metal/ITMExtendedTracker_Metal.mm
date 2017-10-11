// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMExtendedTracker_Metal.h"
#include "../Shared/ITMExtendedTracker_Shared.h"
#import <Accelerate/Accelerate.h>

using namespace ITMLib;

struct ExtendedTracker_metalBits
{
    id<MTLFunction> f_extendedTrackerOneLevel_rt_device;
    id<MTLComputePipelineState> p_extendedTrackerOneLevel_rt_device;

    id<MTLBuffer> paramsBuffer;
};

static ExtendedTracker_metalBits et_metalBits;

ITMExtendedTracker_Metal::ITMExtendedTracker_Metal(Vector2i imgSize_d,
												   Vector2i imgSize_rgb,
												   bool useDepth,
												   bool useColour,
												   float colourWeight,
												   TrackerIterationType *trackingRegime,
												   int noHierarchyLevels,
												   float terminationThreshold,
												   float failureDetectorThreshold,
												   float viewFrustum_min,
												   float viewFrustum_max,
												   float minColourGradient,
												   int tukeyCutOff,
												   int framesToSkip,
												   int framesToWeight,
												   const ITMLowLevelEngine *lowLevelEngine)
: ITMExtendedTracker(imgSize_d,
					 imgSize_rgb,
					 useDepth,
					 useColour,
					 colourWeight,
					 trackingRegime,
					 noHierarchyLevels,
					 terminationThreshold,
					 failureDetectorThreshold,
					 viewFrustum_min,
					 viewFrustum_max,
					 minColourGradient,
					 tukeyCutOff,
					 framesToSkip,
					 framesToWeight,
					 lowLevelEngine,
					 MEMORYDEVICE_CPU)
{
    allocImgSize = imgSize;

    allocateMetalData((void**)&ATb_metal, (void**)&ATb_metal_mb, allocImgSize.x * allocImgSize.y * 6 * sizeof(float), true);
    allocateMetalData((void**)&ATA_metal, (void**)&ATA_metal_mb, allocImgSize.x * allocImgSize.y * 21 * sizeof(float), true);
    allocateMetalData((void**)&noValidPoints_metal, (void**)&noValidPoints_metal_mb, allocImgSize.x * allocImgSize.y * sizeof(float), true);
    allocateMetalData((void**)&f_metal, (void**)&f_metal_mb, allocImgSize.x * allocImgSize.y * sizeof(float), true);

    NSError *errors;
    et_metalBits.f_extendedTrackerOneLevel_rt_device = [[[MetalContext instance]library]newFunctionWithName:@"extendedTrackerOneLevel_g_rt_device"];
    et_metalBits.p_extendedTrackerOneLevel_rt_device = [[[MetalContext instance]device]newComputePipelineStateWithFunction:et_metalBits.f_extendedTrackerOneLevel_rt_device error:&errors];

    et_metalBits.paramsBuffer = BUFFEREMPTY(16384);
}

ITMExtendedTracker_Metal::~ITMExtendedTracker_Metal(void)
{
    freeMetalData((void**)&ATb_metal, (void**)&ATb_metal_mb, allocImgSize.x * allocImgSize.y * 6 * sizeof(float), true);
    freeMetalData((void**)&ATA_metal, (void**)&ATA_metal_mb, allocImgSize.x * allocImgSize.y * 21 * sizeof(float), true);
    freeMetalData((void**)&noValidPoints_metal, (void**)&noValidPoints_metal_mb, allocImgSize.x * allocImgSize.y * sizeof(float), true);
    freeMetalData((void**)&f_metal, (void**)&f_metal_mb, allocImgSize.x * allocImgSize.y * sizeof(float), true);
}

int ITMExtendedTracker_Metal::ComputeGandH_Depth(float &f, float *nabla, float *hessian, Matrix4f approxInvPose)
{
    Vector4f sceneIntrinsics = sceneHierarchyLevel_Depth->intrinsics;
    Vector2i sceneImageSize = sceneHierarchyLevel_Depth->pointsMap->noDims;
    Vector4f viewIntrinsics = viewHierarchyLevel_Depth->intrinsics;
    Vector2i viewImageSize = viewHierarchyLevel_Depth->depth->noDims;

    if (currentIterationType == TRACKER_ITERATION_NONE) return 0;

    bool shortIteration = currentIterationType == TRACKER_ITERATION_ROTATION
						  || currentIterationType == TRACKER_ITERATION_TRANSLATION;

    float sumHessian[6 * 6], sumNabla[6], sumF; int noValidPoints;
    int noPara = shortIteration ? 3 : 6, noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

    noValidPoints = 0; sumF = 0.0f;
    memset(sumHessian, 0, sizeof(float) * noParaSQ);
    memset(sumNabla, 0, sizeof(float) * noPara);

    int viewImageTotalSize = viewImageSize.x * viewImageSize.y;
    int pointsPerThread = 4;

    id<MTLCommandBuffer> commandBuffer = [[[MetalContext instance]commandQueue]commandBuffer];
    id<MTLComputeCommandEncoder> commandEncoder = [commandBuffer computeCommandEncoder];

    ExtendedTrackerOneLevel_rt_Params *params = (ExtendedTrackerOneLevel_rt_Params*)[et_metalBits.paramsBuffer contents];
    params->approxInvPose = approxInvPose; params->sceneIntrinsics = sceneIntrinsics;
    params->sceneImageSize = sceneImageSize; params->scenePose = scenePose;
    params->viewIntrinsics = viewIntrinsics; params->viewImageSize = viewImageSize;
    params->others1.x = spaceThresh[currentLevelId];
    params->others1.y = (float)currentIterationType;
    params->others1.z = (float)pointsPerThread;
    params->others1.w = viewFrustum_min;
    params->others2.x = viewFrustum_max;
    params->others2.y = tukeyCutOff;
    params->others2.z = framesToSkip;
    params->others2.w = framesToWeight;
    params->others3.x = currentFrameNo;

    [commandEncoder setComputePipelineState:et_metalBits.p_extendedTrackerOneLevel_rt_device];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) noValidPoints_metal_mb                               offset:0 atIndex:0];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) f_metal_mb                                           offset:0 atIndex:1];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) ATA_metal_mb                                         offset:0 atIndex:2];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) ATb_metal_mb                                         offset:0 atIndex:3];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) viewHierarchyLevel_Depth->depth->GetMetalBuffer()          offset:0 atIndex:4];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) sceneHierarchyLevel_Depth->pointsMap->GetMetalBuffer()     offset:0 atIndex:5];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) sceneHierarchyLevel_Depth->normalsMap->GetMetalBuffer()    offset:0 atIndex:6];
    [commandEncoder setBuffer:et_metalBits.paramsBuffer                                                     offset:0 atIndex:7];

    MTLSize blockSize = {4, 4, 1};
    MTLSize gridSize = {(NSUInteger)ceil(((float)viewImageSize.x / pointsPerThread) / (float)blockSize.width),
        (NSUInteger)ceil(((float)viewImageSize.y / pointsPerThread) / (float)blockSize.height), 1};

    memset(noValidPoints_metal, 0, sizeof(float) * viewImageTotalSize);
    memset(f_metal, 0, sizeof(float) * viewImageTotalSize);

    [commandEncoder dispatchThreadgroups:gridSize threadsPerThreadgroup:blockSize];
    [commandEncoder endEncoding];

    [commandBuffer commit];

    [commandBuffer waitUntilCompleted];

    for (int locId = 0; locId < (viewImageTotalSize / (pointsPerThread * pointsPerThread)); locId++)
    {
        if (noValidPoints_metal[locId] > 0)
        {
            noValidPoints += noValidPoints_metal[locId];
            sumF += f_metal[locId];
            for (int i = 0; i < noPara; i++) sumNabla[i] += ATb_metal[i + noPara * locId];
            for (int i = 0; i < noParaSQ; i++) sumHessian[i] += ATA_metal[i + noParaSQ * locId];
        }
    }

    for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++) hessian[r + c * 6] = sumHessian[counter];
    for (int r = 0; r < noPara; ++r) for (int c = r + 1; c < noPara; c++) hessian[r + c * 6] = hessian[c + r * 6];

    memcpy(nabla, sumNabla, noPara * sizeof(float));
    f = (noValidPoints > 100) ? sqrt(sumF) / noValidPoints : 1e5f;

    return noValidPoints;
}

int ITMExtendedTracker_CPU::ComputeGandH_RGB(float &f, float *nabla, float *hessian, Matrix4f approxInvPose)
{
	// TODO: not currently implemented
	return 0;
}

void ITMExtendedTracker_CPU::ProjectCurrentIntensityFrame(ITMFloat4Image *points_out,
														  ITMFloatImage *intensity_out,
														  const ITMFloatImage *intensity_in,
														  const ITMFloatImage *depth_in,
														  const Vector4f &intrinsics_depth,
														  const Vector4f &intrinsics_rgb,
														  const Matrix4f &scenePose)
{
	// TODO: not currently implemented
}
