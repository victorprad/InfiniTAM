// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMSurfelSceneReconstructionEngine.h"

namespace ITMLib
{
  /**
   * \brief An instance of an instantiation of this class template can be used to make a surfel-based reconstruction of a 3D scene using the CPU.
   */
  template <typename TSurfel>
  class ITMSurfelSceneReconstructionEngine_CPU : public ITMSurfelSceneReconstructionEngine<TSurfel>
  {
    //#################### PUBLIC MEMBER FUNCTIONS ####################
  public:
    /** Override */
    virtual void AllocateSceneFromDepth(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const;

    /** Override */
    virtual void IntegrateIntoScene(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const;

    /** Override */
    virtual void ResetScene(ITMSurfelScene<TSurfel> *scene) const;

    //#################### PRIVATE MEMBER FUNCTIONS ####################
  private:
    /** Override */
    virtual void PreprocessDepthMap(const ITMView *view) const;
  };
}
