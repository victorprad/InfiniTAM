// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMViewBuilderFactory.h"
using namespace ORUtils;

#include "CPU/ITMViewBuilder_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/ITMViewBuilder_CUDA.h"
#endif

namespace ITMLib
{

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

ITMViewBuilder *ITMViewBuilderFactory::MakeViewBuilder(const ITMRGBDCalib& calib, DeviceType deviceType)
{
  ITMViewBuilder *viewBuilder = NULL;

  switch(deviceType)
  {
    case DEVICE_CPU:
      viewBuilder = new ITMViewBuilder_CPU(calib);
      break;
    case DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
      viewBuilder = new ITMViewBuilder_CUDA(calib);
#endif
      break;
    case DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
      viewBuilder = new ITMViewBuilder_CPU(calib);
#endif
      break;
  }

  return viewBuilder;
}

}
