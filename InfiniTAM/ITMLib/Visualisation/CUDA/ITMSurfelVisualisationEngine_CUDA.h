// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMSurfelVisualisationEngine.h"

namespace ITMLib
{
  /**
   * \brief TODO
   */
  template <typename TSurfel>
  class ITMSurfelVisualisationEngine_CUDA : public ITMSurfelVisualisationEngine<TSurfel>
  {
    //#################### TYPEDEFS & USINGS ####################
  private:
    typedef ITMSurfelVisualisationEngine<TSurfel> Base;
    using typename Base::RenderImageType;

    //#################### PUBLIC MEMBER FUNCTIONS ####################
  public:
#if DEBUG_CORRESPONDENCES
    /** Override */
    virtual void CopyCorrespondencesToBuffer(const ITMSurfelScene<TSurfel> *scene, float *correspondences) const;
#endif

    /** Override */
    virtual void CopySceneToBuffers(const ITMSurfelScene<TSurfel> *scene, float *positions, unsigned char *normals, unsigned char *colours) const;

    /** Override */
    virtual void RenderDepthImage(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMSurfelRenderState *renderState,
                                  ITMFloatImage *outputImage) const;

    /** Override */
    virtual void RenderImage(const ITMSurfelScene<TSurfel> *scene, const ITMSurfelRenderState *renderState,
                             ITMUChar4Image *outputImage, RenderImageType type) const;

    //#################### PRIVATE MEMBER FUNCTIONS ####################
  private:
    /** Override */
    virtual MemoryDeviceType GetMemoryType() const;

    /** Override */
    virtual void MakeIndexImage(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
                                int width, int height, int scaleFactor, unsigned int *surfelIndexImage, int *depthBuffer) const;
  };
}
