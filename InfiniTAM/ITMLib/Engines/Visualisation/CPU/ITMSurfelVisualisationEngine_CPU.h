// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMSurfelVisualisationEngine.h"

namespace ITMLib
{
  /**
   * \brief TODO
   */
  template <typename TSurfel>
  class ITMSurfelVisualisationEngine_CPU : public ITMSurfelVisualisationEngine<TSurfel>
  {
    //#################### TYPEDEFS & USINGS ####################
  private:
    typedef ITMSurfelVisualisationEngine<TSurfel> Base;
    using typename Base::RenderImageType;

    //#################### PUBLIC MEMBER FUNCTIONS ####################
  public:
    /** Override */
    virtual void CopyCorrespondencesToBuffers(const ITMSurfelScene<TSurfel> *scene, float *newPositions, float *oldPositions, float *correspondences) const;

    /** Override */
    virtual void CopySceneToBuffers(const ITMSurfelScene<TSurfel> *scene, float *positions, unsigned char *normals, unsigned char *colours) const;

    /** Override */
    virtual void CreateICPMaps(const ITMSurfelScene<TSurfel> *scene, const ITMSurfelRenderState *renderState, ITMTrackingState *trackingState) const;

    /** Override */
    virtual void RenderDepthImage(const ITMSurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose, const ITMSurfelRenderState *renderState,
                                  ITMFloatImage *outputImage) const;

    /** Override */
    virtual void RenderImage(const ITMSurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose, const ITMSurfelRenderState *renderState,
                             ITMUChar4Image *outputImage, RenderImageType type) const;

    //#################### PRIVATE MEMBER FUNCTIONS ####################
  private:
    /** Override */
    virtual MemoryDeviceType GetMemoryType() const;

    /** Override */
    virtual void MakeIndexImage(const ITMSurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
                                int width, int height, int scaleFactor, unsigned int *surfelIndexImage, bool useRadii,
                                UnstableSurfelRenderingMode unstableSurfelRenderingMode, int *depthBuffer) const;
  };
}
