// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM
#pragma once

#ifdef __OBJC__

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>
#include <mach/mach.h>
#include <sys/mman.h>

#ifndef BUFFERNOCOPY
#define BUFFERNOCOPY(x, marime) [[[ITMMetalContext instance]device] newBufferWithBytesNoCopy:(x) length:marime options:MTLResourceOptionCPUCacheModeDefault deallocator:nil]
#endif

#ifndef BUFFERCOPY
#define BUFFERCOPY(x, marime) [[[ITMMetalContext instance]device] newBufferWithBytes:(x) length:marime options:MTLResourceOptionCPUCacheModeDefault]
#endif

#ifndef BUFFEREMPTY
#define BUFFEREMPTY(marime) [[[ITMMetalContext instance]device] newBufferWithLength:marime options:MTLResourceOptionCPUCacheModeDefault]
#endif

@protocol MTLDevice, MTLLibrary, MTLCommandQueue;

@interface ITMMetalContext : NSObject

@property (strong) id<MTLDevice> device;
@property (strong) id<MTLLibrary> library;
@property (strong) id<MTLCommandQueue> commandQueue;
@property (strong) id<MTLCommandBuffer> commandBuffer;

+(ITMMetalContext *) instance;
+(int)roundUpTo16384 : (int) size;

@end

#endif

void allocateMetalData(void **data, void **metalBuffer, int size, bool roundUp);
void freeMetalData(void **data, void **metalBufber, int size, bool roundUp);
