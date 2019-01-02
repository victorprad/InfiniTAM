// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMLowLevelEngineFactory.h"
using namespace ORUtils;

#include "CPU/ITMLowLevelEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/ITMLowLevelEngine_CUDA.h"
#endif

namespace ITMLib
{

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

ITMLowLevelEngine *ITMLowLevelEngineFactory::MakeLowLevelEngine(DeviceType deviceType)
{
  ITMLowLevelEngine *lowLevelEngine = NULL;

  switch(deviceType)
  {
    case DEVICE_CPU:
      lowLevelEngine = new ITMLowLevelEngine_CPU();
      break;
    case DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
      lowLevelEngine = new ITMLowLevelEngine_CUDA();
#endif
      break;
    case DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
      lowLevelEngine = new ITMLowLevelEngine_CPU();
#endif
      break;
  }

  return lowLevelEngine;
}

}
