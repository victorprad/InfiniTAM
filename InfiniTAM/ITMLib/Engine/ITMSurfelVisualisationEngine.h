// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Objects/ITMIntrinsics.h"
#include "../Objects/ITMPose.h"
#include "../Objects/ITMSurfelRenderState.h"
#include "../Objects/ITMSurfelScene.h"

namespace ITMLib
{
  /**
   * \brief TODO
   */
  template <typename TSurfel>
  class ITMSurfelVisualisationEngine
  {
    //#################### ENUMERATIONS ####################
  public:
    /**
     * \brief TODO
     */
    enum RenderImageType
    {
      RENDER_LAMBERTIAN,
      RENDER_COLOUR
    };

    //#################### PUBLIC MEMBER FUNCTIONS ####################
  public:
    /**
     * \brief TODO
     */
    virtual void CopySceneToBuffers(const ITMSurfelScene<TSurfel> *scene, float *positions) const = 0;

    /**
     * \brief TODO
     */
    virtual void FindSurface(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
                             const ITMSurfelRenderState *renderState) const = 0;

    /**
     * \brief TODO
     */
    virtual void RenderImage(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
                             const ITMSurfelRenderState *renderState, ITMUChar4Image *outputImage, RenderImageType type = RENDER_LAMBERTIAN) const = 0;
  };
}
