// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#pragma once

#include "Interface/ITMSurfelSceneReconstructionEngine.h"
#include "../../Utils/ITMLibSettings.h"

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
     * \param depthImageSize  The size of the depth images that will be fused into the scene.
     * \param deviceType      The device on which the surfel scene reconstruction engine should operate.
     * \return                The surfel scene reconstruction engine.
     */
    static ITMSurfelSceneReconstructionEngine<TSurfel> *make_surfel_scene_reconstruction_engine(const Vector2i& depthImageSize, ITMLib::ITMLibSettings::DeviceType deviceType);
  };
}
