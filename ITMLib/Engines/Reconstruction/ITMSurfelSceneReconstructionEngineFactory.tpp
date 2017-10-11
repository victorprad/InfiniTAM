// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#include "ITMSurfelSceneReconstructionEngineFactory.h"

#include "CPU/ITMSurfelSceneReconstructionEngine_CPU.h"

#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/ITMSurfelSceneReconstructionEngine_CUDA.h"
#endif

namespace ITMLib
{

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

template <typename TSurfel>
ITMSurfelSceneReconstructionEngine<TSurfel> *
ITMSurfelSceneReconstructionEngineFactory<TSurfel>::make_surfel_scene_reconstruction_engine(const Vector2i& depthImageSize, ITMLibSettings::DeviceType deviceType)
{
  ITMSurfelSceneReconstructionEngine<TSurfel> *reconstructionEngine = NULL;

  if(deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifndef COMPILE_WITHOUT_CUDA
    reconstructionEngine = new ITMSurfelSceneReconstructionEngine_CUDA<TSurfel>(depthImageSize);
#else
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    reconstructionEngine = new ITMSurfelSceneReconstructionEngine_CPU<TSurfel>(depthImageSize);
  }

  return reconstructionEngine;
}

}
