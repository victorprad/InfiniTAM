// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Objects/ITMSurfelScene.h"
#include "../Objects/ITMTrackingState.h"
#include "../Objects/ITMView.h"

namespace ITMLib
{
  /**
   * \brief An instance of an instantiation of this class template can be used to make a surfel-based reconstruction of a 3D scene.
   */
  template <typename TSurfel>
  class ITMSurfelSceneReconstructionEngine
  {
    //#################### PUBLIC MEMBER FUNCTIONS ####################
  public:
    /**
     * \brief TODO
     */
    virtual void AllocateSceneFromDepth(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const = 0;

    /**
     * \brief Updates the specified surfel-based scene by integrating depth and possibly colour information from the given view.
     *
     * \param scene         The scene to update.
     * \param view          The current view (containing the live input images from the current image source).
     * \param trackingState The current tracking state.
     */
    virtual void IntegrateIntoScene(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const = 0;

    /**
     * \brief Resets the specified surfel-based scene.
     *
     * \param scene The scene to reset.
     */
    virtual void ResetScene(ITMSurfelScene<TSurfel> *scene) const = 0;
  };
}
