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

    /** The factor by which to supersample (in each axis) the index image used for finding surfel correspondences. */
    int supersamplingFactor;

    /** The maximum depth a surfel must have in order for it to be used for tracking. */
    float trackingSurfelMaxDepth;

    /** The minimum confidence value a surfel must have in order for it to be used for tracking. */
    float trackingSurfelMinConfidence;

    /** The number of time steps a surfel is allowed to be unstable without being updated before being removed. */
    int unstableSurfelPeriod;

    //#################### CONSTRUCTORS ####################

    /**
     * \brief Constructs a set of surfel scene parameters.
     *
     * \param stableSurfelConfidence_       The confidence value a surfel must have in order for it to be considered "stable".
     * \param supersamplingFactor_          The factor by which to supersample (in each axis) the index image used for finding surfel correspondences.
     * \param trackingSurfelMaxDepth_       The maximum depth a surfel must have in order for it to be used for tracking.
     * \param trackingSurfelMinConfidence_  The minimum confidence value a surfel must have in order for it to be used for tracking.
     * \param unstableSurfelPeriod_         The number of time steps a surfel is allowed to be unstable without being updated before being removed.
     */
    explicit ITMSurfelSceneParams(float stableSurfelConfidence_, int supersamplingFactor_, float trackingSurfelMaxDepth_,
                                  float trackingSurfelMinConfidence_, int unstableSurfelPeriod_)
    : stableSurfelConfidence(stableSurfelConfidence_),
      supersamplingFactor(supersamplingFactor_),
      trackingSurfelMaxDepth(trackingSurfelMaxDepth_),
      trackingSurfelMinConfidence(trackingSurfelMinConfidence_),
      unstableSurfelPeriod(unstableSurfelPeriod_)
    {}
  };
}
