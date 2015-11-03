// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMSurfelSceneReconstructionEngine.h"
#include "../Utils/ITMLibSettings.h"

namespace ITMLib
{
  /**
   * \brief An instantiation of this struct can be used to construct surfel scene reconstruction engines.
   */
  template <typename TSurfel>
  struct ITMSurfelSceneReconstructionEngineFactory
  {
    //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

    /**
     * \brief Makes a surfel scene reconstruction engine.
     *
     * \param deviceType  TODO
     * \return            TODO
     */
    static ITMSurfelSceneReconstructionEngine<TSurfel> *make_surfel_scene_reconstruction_engine(const Vector2i& depthImageSize, ITMLib::ITMLibSettings::DeviceType deviceType);
  };
}
