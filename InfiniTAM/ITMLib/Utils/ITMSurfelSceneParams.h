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

    /** The maximum fraction by which a new surfel can have a larger radius than the surfel into which it is being fused if full fusion is to occur. */
    float deltaRadius;

    /** The sigma value for the Gaussian used when calculating the sample confidence. */
    float gaussianConfidenceSigma;

    /** The maximum angle allowed between the normals of a pair of surfels if they are to be merged. */
    float maxMergeAngle;

    /** The maximum distance allowed between a pair of surfels if they are to be merged. */
    float maxMergeDist;

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

    /** Whether or not to use a Gaussian-weighted sample confidence as described in the Keller paper. */
    bool useGaussianSampleConfidence;

    //#################### CONSTRUCTORS ####################

    /**
     * \brief Constructs a set of surfel scene parameters.
     *
     * \param deltaRadius_                  The maximum fraction by which a new surfel can have a larger radius than the surfel into which it is being fused if full fusion is to occur.
     * \param gaussianConfidenceSigma_      The sigma value for the Gaussian used when calculating the sample confidence.
     * \param maxMergeAngle_                The maximum angle allowed between the normals of a pair of surfels if they are to be merged.
     * \param maxMergeDist_                 The maximum distance allowed between a pair of surfels if they are to be merged.
     * \param stableSurfelConfidence_       The confidence value a surfel must have in order for it to be considered "stable".
     * \param supersamplingFactor_          The factor by which to supersample (in each axis) the index image used for finding surfel correspondences.
     * \param trackingSurfelMaxDepth_       The maximum depth a surfel must have in order for it to be used for tracking.
     * \param trackingSurfelMinConfidence_  The minimum confidence value a surfel must have in order for it to be used for tracking.
     * \param unstableSurfelPeriod_         The number of time steps a surfel is allowed to be unstable without being updated before being removed.
     * \param useGaussianSampleConfidence_  Whether or not to use a Gaussian-weighted sample confidence as described in the Keller paper.
     */
    explicit ITMSurfelSceneParams(float deltaRadius_, float gaussianConfidenceSigma_, float maxMergeAngle_, float maxMergeDist_, float stableSurfelConfidence_,
                                  int supersamplingFactor_, float trackingSurfelMaxDepth_, float trackingSurfelMinConfidence_, int unstableSurfelPeriod_,
                                  bool useGaussianSampleConfidence_)
    : deltaRadius(deltaRadius_),
      gaussianConfidenceSigma(gaussianConfidenceSigma_),
      maxMergeAngle(maxMergeAngle_),
      maxMergeDist(maxMergeDist_),
      stableSurfelConfidence(stableSurfelConfidence_),
      supersamplingFactor(supersamplingFactor_),
      trackingSurfelMaxDepth(trackingSurfelMaxDepth_),
      trackingSurfelMinConfidence(trackingSurfelMinConfidence_),
      unstableSurfelPeriod(unstableSurfelPeriod_),
      useGaussianSampleConfidence(useGaussianSampleConfidence_)
    {}
  };
}
