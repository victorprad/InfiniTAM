// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMSurfelVisualisationEngine.h"

namespace ITMLib
{
  /**
   * \brief TODO
   */
  template <typename TSurfel>
  class ITMSurfelVisualisationEngine_CUDA : public ITMSurfelVisualisationEngine<TSurfel>
  {
    //#################### USINGS ####################
  private:
    using typename ITMSurfelVisualisationEngine<TSurfel>::RenderImageType;

    //#################### PUBLIC MEMBER FUNCTIONS ####################
  public:
    /** Override */
    virtual void CopySceneToBuffers(const ITMSurfelScene<TSurfel> *scene, float *positions, unsigned char *colours) const;

    /** Override */
    virtual void FindSurface(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
                             const ITMSurfelRenderState *renderState) const;

    /** Override */
    virtual void RenderImage(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
                             const ITMSurfelRenderState *renderState, ITMUChar4Image *outputImage, RenderImageType type) const;
  };
}
