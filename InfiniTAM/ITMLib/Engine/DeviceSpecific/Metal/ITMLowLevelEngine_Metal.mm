// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#ifdef COMPILE_WITH_METAL

#import "MetalContext.h"

#include "ITMLowLevelEngine_Metal.h"
#include "../../DeviceAgnostic/ITMLowLevelEngine.h"

id<MTLFunction> f_convertDisparityToDepth;
id<MTLComputePipelineState> p_convertDisparityToDepth;

id<MTLBuffer> paramsBuffer_LowLevelEngine;

using namespace ITMLib::Engine;

ITMLowLevelEngine_Metal::ITMLowLevelEngine_Metal(void)
: ITMLowLevelEngine_CPU()
{
    NSError *errors;
    f_convertDisparityToDepth = [[[MetalContext instance]library]newFunctionWithName:@"convertDisparityToDepth_device"];
    p_convertDisparityToDepth = [[[MetalContext instance]device]newComputePipelineStateWithFunction:f_convertDisparityToDepth error:&errors];
    
    paramsBuffer_LowLevelEngine = BUFFEREMPTY(16384);
}

ITMLowLevelEngine_Metal::~ITMLowLevelEngine_Metal(void) { }

void ITMLowLevelEngine_Metal::ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const ITMIntrinsics *depthIntrinsics,
                                                    const ITMDisparityCalib *disparityCalib)
{
    id<MTLCommandBuffer> commandBuffer = [[[MetalContext instance]commandQueue]commandBuffer];
    id<MTLComputeCommandEncoder> commandEncoder = [commandBuffer computeCommandEncoder];
    
    id<MTLBuffer> d_out_buffer = (__bridge id<MTLBuffer>) depth_out->GetMetalBuffer();
    id<MTLBuffer> d_in_buffer = (__bridge id<MTLBuffer>) depth_in->GetMetalBuffer();
    
    ConvertDisparityToDepth_Params *params = (ConvertDisparityToDepth_Params*)[paramsBuffer_LowLevelEngine contents];
    params->fx_depth = depthIntrinsics->projectionParamsSimple.fx;
    params->imgSize = depth_in->noDims;
    params->disparityCalibParams = disparityCalib->params;
    
    [commandEncoder setComputePipelineState:p_convertDisparityToDepth];
    [commandEncoder setBuffer:d_out_buffer offset:0 atIndex:0];
    [commandEncoder setBuffer:d_in_buffer offset:0 atIndex:1];
    [commandEncoder setBuffer:paramsBuffer_LowLevelEngine offset:0 atIndex:2];
    
    MTLSize blockSize = {8, 8, 1};
    MTLSize gridSize = {(NSUInteger)ceil((float)depth_in->noDims.x / (float)blockSize.width),
        (NSUInteger)ceil((float)depth_in->noDims.y / (float)blockSize.height), 1};
    
    [commandEncoder dispatchThreadgroups:gridSize threadsPerThreadgroup:blockSize];
    [commandEncoder endEncoding];
    
    [commandBuffer commit];
    
    [commandBuffer waitUntilCompleted];
}

#endif
