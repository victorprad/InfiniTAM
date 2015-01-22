// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#ifdef COMPILE_WITH_METAL

#import "MetalContext.h"

#include "ITMLowLevelEngine_Metal.h"
#include "../../DeviceAgnostic/ITMLowLevelEngine.h"

using namespace ITMLib::Engine;

ITMLowLevelEngine_Metal::ITMLowLevelEngine_Metal(void)
: ITMLowLevelEngine_CPU() { }
ITMLowLevelEngine_Metal::~ITMLowLevelEngine_Metal(void) { }

#endif