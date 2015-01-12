// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM
#ifdef COMPILE_WITH_METAL

#import "ITMMetalContext.h"

#include "ITMLowLevelEngine_Metal.h"
#include "../../DeviceAgnostic/ITMLowLevelEngine.h"

id<MTLFunction> f_convertDisparityToDepth;
id<MTLComputePipelineState> p_convertDisparityToDepth;

id<MTLBuffer> paramsBuffer_LowLevelEngine;

using namespace ITMLib::Engine;

ITMLowLevelEngine_Metal::ITMLowLevelEngine_Metal(void)
{
    NSError *errors;
    f_convertDisparityToDepth = [[[ITMMetalContext instance]library]newFunctionWithName:@"convertDisparityToDepth_device"];
    p_convertDisparityToDepth = [[[ITMMetalContext instance]device]newComputePipelineStateWithFunction:f_convertDisparityToDepth error:&errors];
    
    paramsBuffer_LowLevelEngine = BUFFEREMPTY(16384);
}

ITMLowLevelEngine_Metal::~ITMLowLevelEngine_Metal(void) { }

void ITMLowLevelEngine_Metal::CopyImage(ITMUChar4Image *image_out, const ITMUChar4Image *image_in)
{
    Vector4u *dest = image_out->GetData(false);
    const Vector4u *src = image_in->GetData(false);
    
    memcpy(dest, src, image_in->dataSize * sizeof(Vector4u));
}

void ITMLowLevelEngine_Metal::CopyImage(ITMFloatImage *image_out, const ITMFloatImage *image_in)
{
    float *dest = image_out->GetData(false);
    const float *src = image_in->GetData(false);
    
    memcpy(dest, src, image_in->dataSize * sizeof(float));
}

void ITMLowLevelEngine_Metal::CopyImage(ITMFloat4Image *image_out, const ITMFloat4Image *image_in)
{
    Vector4f *dest = image_out->GetData(false);
    const Vector4f *src = image_in->GetData(false);
    
    memcpy(dest, src, image_in->dataSize * sizeof(Vector4f));
}

void ITMLowLevelEngine_Metal::FilterSubsample(ITMUChar4Image *image_out, const ITMUChar4Image *image_in)
{
    Vector2i oldDims = image_in->noDims;
    Vector2i newDims; newDims.x = image_in->noDims.x / 2; newDims.y = image_in->noDims.y / 2;
    
    image_out->ChangeDims(newDims);
    
    const Vector4u *imageData_in = image_in->GetData(false);
    Vector4u *imageData_out = image_out->GetData(false);
    
    for (int y = 0; y < newDims.y; y++) for (int x = 0; x < newDims.x; x++)
        filterSubsample(imageData_out, x, y, newDims, imageData_in, oldDims);
}

void ITMLowLevelEngine_Metal::FilterSubsampleWithHoles(ITMFloatImage *image_out, const ITMFloatImage *image_in)
{
    Vector2i oldDims = image_in->noDims;
    Vector2i newDims; newDims.x = image_in->noDims.x / 2; newDims.y = image_in->noDims.y / 2;
    
    image_out->ChangeDims(newDims);
    
    const float *imageData_in = image_in->GetData(false);
    float *imageData_out = image_out->GetData(false);
    
    for (int y = 0; y < newDims.y; y++) for (int x = 0; x < newDims.x; x++)
        filterSubsampleWithHoles(imageData_out, x, y, newDims, imageData_in, oldDims);
}

void ITMLowLevelEngine_Metal::FilterSubsampleWithHoles(ITMFloat4Image *image_out, const ITMFloat4Image *image_in)
{
    Vector2i oldDims = image_in->noDims;
    Vector2i newDims; newDims.x = image_in->noDims.x / 2; newDims.y = image_in->noDims.y / 2;
    
    image_out->ChangeDims(newDims);
    
    const Vector4f *imageData_in = image_in->GetData(false);
    Vector4f *imageData_out = image_out->GetData(false);
    
    for (int y = 0; y < newDims.y; y++) for (int x = 0; x < newDims.x; x++)
        filterSubsampleWithHoles(imageData_out, x, y, newDims, imageData_in, oldDims);
}

void ITMLowLevelEngine_Metal::GradientX(ITMShort4Image *grad_out, const ITMUChar4Image *image_in)
{
    grad_out->ChangeDims(image_in->noDims);
    Vector2i imgSize = image_in->noDims;
    
    Vector4s *grad = grad_out->GetData(false);
    const Vector4u *image = image_in->GetData(false);
    
    memset(grad, 0, imgSize.x * imgSize.y * sizeof(Vector3s));
    
    for (int y = 1; y < imgSize.y - 1; y++) for (int x = 1; x < imgSize.x - 1; x++)
        gradientX(grad, x, y, image, imgSize);
}

void ITMLowLevelEngine_Metal::GradientY(ITMShort4Image *grad_out, const ITMUChar4Image *image_in)
{
    grad_out->ChangeDims(image_in->noDims);
    Vector2i imgSize = image_in->noDims;
    
    Vector4s *grad = grad_out->GetData(false);
    const Vector4u *image = image_in->GetData(false);
    
    memset(grad, 0, imgSize.x * imgSize.y * sizeof(Vector3s));
    
    for (int y = 1; y < imgSize.y - 1; y++) for (int x = 1; x < imgSize.x - 1; x++)
        gradientY(grad, x, y, image, imgSize);
}

void ITMLowLevelEngine_Metal::ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const ITMIntrinsics *depthIntrinsics,
                                                    const ITMDisparityCalib *disparityCalib)
{
    id<MTLCommandBuffer> commandBuffer = [[[ITMMetalContext instance]commandQueue]commandBuffer];
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

void ITMLowLevelEngine_Metal::ConvertDepthMMToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in)
{
    Vector2i imgSize = depth_in->noDims;
    
    const short *d_in = depth_in->GetData(false);
    float *d_out = depth_out->GetData(false);
    
    for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
        convertDepthMMToFloat(d_out, x, y, d_in, imgSize);
}

#endif
