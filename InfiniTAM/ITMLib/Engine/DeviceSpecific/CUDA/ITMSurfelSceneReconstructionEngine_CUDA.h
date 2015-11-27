// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMSurfelSceneReconstructionEngine.h"

namespace ITMLib
{
  /**
   * \brief An instance of an instantiation of this class template can be used to make a surfel-based reconstruction of a 3D scene using CUDA.
   */
  template <typename TSurfel>
  class ITMSurfelSceneReconstructionEngine_CUDA : public ITMSurfelSceneReconstructionEngine<TSurfel>
  {
    //#################### CONSTRUCTORS ####################
  public:
    /**
     * \brief TODO
     */
    explicit ITMSurfelSceneReconstructionEngine_CUDA(const Vector2i& depthImageSize);

    //#################### PRIVATE MEMBER FUNCTIONS ####################
  private:
    /** Override */
    virtual void AddNewSurfels(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const;

    /** Override */
    virtual void FindCorrespondingSurfels(const ITMSurfelScene<TSurfel> *scene, const ITMView *view) const;

    /** Override */
    virtual void FuseMatchedPoints(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const;

    /** Override */
    virtual void GenerateIndexMap(const ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMPose& pose) const;

    /** Override */
    virtual void PreprocessDepthMap(const ITMView *view) const;

    /** Override */
    virtual void RemoveBadSurfels(ITMSurfelScene<TSurfel> *scene) const;
  };
}
