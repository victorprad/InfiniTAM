// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

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

    /** The number of time steps a surfel is allowed to be unstable without being updated before being removed. */
    int unstableSurfelPeriod;

    //#################### CONSTRUCTORS ####################

    /**
     * \brief Constructs a set of surfel scene parameters.
     *
     * \param stableSurfelConfidence_ The confidence value a surfel must have in order for it to be considered "stable".
     */
    explicit ITMSurfelSceneParams(float stableSurfelConfidence_, int unstableSurfelPeriod_)
    : stableSurfelConfidence(stableSurfelConfidence_),
      unstableSurfelPeriod(unstableSurfelPeriod_)
    {}
  };
}
