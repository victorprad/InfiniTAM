// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Objects/Camera/ITMIntrinsics.h"
#include "../../Objects/Camera/ITMPose.h"
#include "../../Objects/RenderStates/ITMSurfelRenderState.h"
#include "../../Objects/Scene/ITMSurfelScene.h"
#include "../../Utils/ITMImageTypes.h"

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

    //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
  public:
#if DEBUG_CORRESPONDENCES
    /**
     * \brief TODO
     */
    virtual void CopyCorrespondencesToBuffer(const ITMSurfelScene<TSurfel> *scene, float *correspondences) const = 0;
#endif

    /**
     * \brief TODO
     */
    virtual void CopySceneToBuffers(const ITMSurfelScene<TSurfel> *scene, float *positions, unsigned char *normals, unsigned char *colours) const = 0;

    /**
     * \brief TODO
     */
    virtual void RenderImage(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
                             const ITMSurfelRenderState *renderState, ITMUChar4Image *outputImage, RenderImageType type = RENDER_LAMBERTIAN) const = 0;

    //#################### PUBLIC MEMBER FUNCTIONS ####################
  public:
    /**
     * \brief TODO
     */
    void FindSurface(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
                     ITMSurfelRenderState *renderState) const;

    /**
     * \brief TODO
     */
    void FindSurfaceSuper(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
                          ITMSurfelRenderState *renderState) const;

    //#################### PRIVATE ABSTRACT MEMBER FUNCTIONS ####################
  private:
    /**
     * \brief TODO
     */
    virtual MemoryDeviceType GetMemoryType() const = 0;

    /**
     * \brief TODO
     */
    virtual void MakeIndexImage(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
                                int width, int height, int scaleFactor, unsigned int *surfelIndexImage, int *depthBuffer) const = 0;
  };
}
