// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#include "ITMSurfelVisualisationEngineFactory.h"

#include "CPU/ITMSurfelVisualisationEngine_CPU.h"

#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/ITMSurfelVisualisationEngine_CUDA.h"
#endif

namespace ITMLib
{

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

template <typename TSurfel>
ITMSurfelVisualisationEngine<TSurfel> *
ITMSurfelVisualisationEngineFactory<TSurfel>::make_surfel_visualisation_engine(ITMLibSettings::DeviceType deviceType)
{
  ITMSurfelVisualisationEngine<TSurfel> *visualisationEngine = NULL;

  if(deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifndef COMPILE_WITHOUT_CUDA
    visualisationEngine = new ITMSurfelVisualisationEngine_CUDA<TSurfel>;
#else
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    visualisationEngine = new ITMSurfelVisualisationEngine_CPU<TSurfel>;
  }

  return visualisationEngine;
}

}
