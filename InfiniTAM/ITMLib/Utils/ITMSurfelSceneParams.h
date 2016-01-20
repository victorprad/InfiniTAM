// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ORUtils/PlatformIndependence.h"

namespace ITMLib
{
  /**
   * \brief An instance of this struct can be used to specify parameters for a surfel scene.
   */
  struct ITMSurfelSceneParams
  {
    //#################### PUBLIC VARIABLES ####################

    /** The confidence value a surfel must have in order for it to be considered "stable". */
    float stableSurfelConfidence;

    //#################### CONSTRUCTORS ####################

    /**
     * \brief Constructs a set of surfel scene parameters.
     *
     * \param stableSurfelConfidence_ The confidence value a surfel must have in order for it to be considered "stable".
     */
    _CPU_AND_GPU_CODE_
    explicit ITMSurfelSceneParams(float stableSurfelConfidence_)
    : stableSurfelConfidence(stableSurfelConfidence_)
    {}
  };
}
