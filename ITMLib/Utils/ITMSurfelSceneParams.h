// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

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

    /** The maximum radius a surfel is allowed to have. */
    float maxSurfelRadius;

    /** The minimum factor by which the radii of a pair of surfels must overlap if they are to be merged. */
    float minRadiusOverlapFactor;

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

    /** The z offset to apply to unstable surfels when trying to ensure that they are only rendered if there is no stable alternative. */
    int unstableSurfelZOffset;

    /** Whether or not to use a Gaussian-weighted sample confidence as described in the Keller paper. */
    bool useGaussianSampleConfidence;

    /** Whether or not to use surfel merging. */
    bool useSurfelMerging;

    //#################### CONSTRUCTORS ####################

    /**
     * \brief Constructs a set of surfel scene parameters.
     *
     * \param deltaRadius_                  The maximum fraction by which a new surfel can have a larger radius than the surfel into which it is being fused if full fusion is to occur.
     * \param gaussianConfidenceSigma_      The sigma value for the Gaussian used when calculating the sample confidence.
     * \param maxMergeAngle_                The maximum angle allowed between the normals of a pair of surfels if they are to be merged.
     * \param maxMergeDist_                 The maximum distance allowed between a pair of surfels if they are to be merged.
     * \param maxSurfelRadius_              The maximum radius a surfel is allowed to have.
     * \param minRadiusOverlapFactor_       The minimum factor by which the radii of a pair of surfels must overlap if they are to be merged.
     * \param stableSurfelConfidence_       The confidence value a surfel must have in order for it to be considered "stable".
     * \param supersamplingFactor_          The factor by which to supersample (in each axis) the index image used for finding surfel correspondences.
     * \param trackingSurfelMaxDepth_       The maximum depth a surfel must have in order for it to be used for tracking.
     * \param trackingSurfelMinConfidence_  The minimum confidence value a surfel must have in order for it to be used for tracking.
     * \param unstableSurfelPeriod_         The number of time steps a surfel is allowed to be unstable without being updated before being removed.
     * \param unstableSurfelZOffset_        The z offset to apply to unstable surfels when trying to ensure that they are only rendered if there is no stable alternative.
     * \param useGaussianSampleConfidence_  Whether or not to use a Gaussian-weighted sample confidence as described in the Keller paper.
     * \param useSurfelMerging_             Whether or not to use surfel merging.
     */
    explicit ITMSurfelSceneParams(float deltaRadius_, float gaussianConfidenceSigma_, float maxMergeAngle_, float maxMergeDist_, float maxSurfelRadius_,
                                  float minRadiusOverlapFactor_, float stableSurfelConfidence_, int supersamplingFactor_, float trackingSurfelMaxDepth_,
                                  float trackingSurfelMinConfidence_, int unstableSurfelPeriod_, int unstableSurfelZOffset_, bool useGaussianSampleConfidence_,
                                  bool useSurfelMerging_)
    : deltaRadius(deltaRadius_),
      gaussianConfidenceSigma(gaussianConfidenceSigma_),
      maxMergeAngle(maxMergeAngle_),
      maxMergeDist(maxMergeDist_),
      maxSurfelRadius(maxSurfelRadius_),
      minRadiusOverlapFactor(minRadiusOverlapFactor_),
      stableSurfelConfidence(stableSurfelConfidence_),
      supersamplingFactor(supersamplingFactor_),
      trackingSurfelMaxDepth(trackingSurfelMaxDepth_),
      trackingSurfelMinConfidence(trackingSurfelMinConfidence_),
      unstableSurfelPeriod(unstableSurfelPeriod_),
      unstableSurfelZOffset(unstableSurfelZOffset_),
      useGaussianSampleConfidence(useGaussianSampleConfidence_),
      useSurfelMerging(useSurfelMerging_)
    {}
  };
}
