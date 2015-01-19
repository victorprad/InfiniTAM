// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#import "MetalContext.h"
#import <Metal/Metal.h>

@implementation MetalContext

+(MetalContext*) instance
{
    static MetalContext *gInstance = NULL;
    @synchronized(self)
    {
        if (gInstance == NULL) gInstance = [[self alloc]initWithDevice:nil];
        return gInstance;
    }
}

+(int)roundUpTo16384:(int)size
{
    float size_f = (float)size;
    float size_div = size_f / 16384.0f;
    float size_ceil = ceilf(size_div);
    return (int)(size_ceil * 16384.0f);
}

- (instancetype)initWithDevice:(id<MTLDevice>)device
{
    if ((self = [super init]))
    {
        _device = device ?: MTLCreateSystemDefaultDevice();
        _library = [_device newDefaultLibrary];
        _commandQueue = [_device newCommandQueue];
        _commandBuffer = [_commandQueue commandBuffer];
    }
    return self;
}

@end

void allocateMetalData(void **data, void **metalBuffer, int size, bool roundUp)
{
    int allocSize;
    if (roundUp) allocSize = [MetalContext roundUpTo16384:size];
    else allocSize = size;
    
    data[0] = mmap(0, allocSize, PROT_READ|PROT_WRITE, MAP_PRIVATE|MAP_ANON, -1, 0);
    metalBuffer[0] = (void*)CFBridgingRetain(BUFFERNOCOPY(data[0], allocSize));
}

void freeMetalData(void **data, void **metalBuffer, int size, bool roundUp)
{
    int allocSize;
    if (roundUp) allocSize = [MetalContext roundUpTo16384:size];
    else allocSize = size;
    
    munmap(data[0], allocSize);
    CFBridgingRelease(metalBuffer[0]);
}