// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "CPU/ITMSceneReconstructionEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/ITMSceneReconstructionEngine_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Metal/ITMSceneReconstructionEngine_Metal.h"
#endif

namespace ITMLib
{

/**
 * \brief This struct provides functions that can be used to construct scene reconstruction engines.
 */
struct ITMSceneReconstructionEngineFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a scene reconstruction engine.
   *
   * \param deviceType  The device on which the scene reconstruction engine should operate.
   */
  template <typename TVoxel, typename TIndex>
  static ITMSceneReconstructionEngine<TVoxel,TIndex> *MakeSceneReconstructionEngine(ITMLibSettings::DeviceType deviceType)
  {
    ITMSceneReconstructionEngine<TVoxel,TIndex> *sceneRecoEngine = NULL;

    switch(deviceType)
    {
      case ITMLibSettings::DEVICE_CPU:
        sceneRecoEngine = new ITMSceneReconstructionEngine_CPU<TVoxel,TIndex>;
        break;
      case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
        sceneRecoEngine = new ITMSceneReconstructionEngine_CUDA<TVoxel,TIndex>;
#endif
        break;
      case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
        sceneRecoEngine = new ITMSceneReconstructionEngine_Metal<TVoxel,TIndex>;
#endif
        break;
    }

    return sceneRecoEngine;
  }
};

}
