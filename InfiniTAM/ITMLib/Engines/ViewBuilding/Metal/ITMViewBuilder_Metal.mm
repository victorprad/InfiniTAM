// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#ifdef COMPILE_WITH_METAL

#import "MetalContext.h"

#include "ITMViewBuilder_Metal.h"
#include "../Shared/ITMViewBuilder_Shared.h"

using namespace ITMLib;

struct ViewBuilder_MetalBits
{
    id<MTLFunction> f_convertDisparityToDepth;
    id<MTLComputePipelineState> p_convertDisparityToDepth;
    
    id<MTLBuffer> paramsBuffer;
};

static ViewBuilder_MetalBits vb_metalBits;

ITMViewBuilder_Metal::ITMViewBuilder_Metal(const ITMRGBDCalib *calib)
: ITMViewBuilder_CPU(calib)
{
    NSError *errors;
    vb_metalBits.f_convertDisparityToDepth = [[[MetalContext instance]library]newFunctionWithName:@"convertDisparityToDepth_device"];
    vb_metalBits.p_convertDisparityToDepth = [[[MetalContext instance]device]newComputePipelineStateWithFunction:vb_metalBits.f_convertDisparityToDepth error:&errors];
    
    vb_metalBits.paramsBuffer = BUFFEREMPTY(16384);
}

ITMViewBuilder_Metal::~ITMViewBuilder_Metal(void) { }

void ITMViewBuilder_Metal::ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *disp_in, const ITMIntrinsics *depthIntrinsics,
                                                   Vector2f disparityCalibParams)
{
    id<MTLCommandBuffer> commandBuffer = [[[MetalContext instance]commandQueue]commandBuffer];
    id<MTLComputeCommandEncoder> commandEncoder = [commandBuffer computeCommandEncoder];
    
    id<MTLBuffer> d_out_buffer = (__bridge id<MTLBuffer>) depth_out->GetMetalBuffer();
    id<MTLBuffer> d_in_buffer = (__bridge id<MTLBuffer>) disp_in->GetMetalBuffer();
    
    ConvertDisparityToDepth_Params *params = (ConvertDisparityToDepth_Params*)[vb_metalBits.paramsBuffer contents];
    params->fx_depth = depthIntrinsics->projectionParamsSimple.fx;
    params->imgSize = disp_in->noDims;
    params->disparityCalibParams = disparityCalibParams ;
    
    [commandEncoder setComputePipelineState:vb_metalBits.p_convertDisparityToDepth];
    [commandEncoder setBuffer:d_out_buffer                              offset:0 atIndex:0];
    [commandEncoder setBuffer:d_in_buffer                               offset:0 atIndex:1];
    [commandEncoder setBuffer:vb_metalBits.paramsBuffer                 offset:0 atIndex:2];
    
    MTLSize blockSize = {16, 16, 1};
    MTLSize gridSize = {(NSUInteger)ceil((float)disp_in->noDims.x / (float)blockSize.width),
        (NSUInteger)ceil((float)disp_in->noDims.y / (float)blockSize.height), 1};
    
    [commandEncoder dispatchThreadgroups:gridSize threadsPerThreadgroup:blockSize];
    [commandEncoder endEncoding];
    
    [commandBuffer commit];
    
    [commandBuffer waitUntilCompleted];
}

#endif
