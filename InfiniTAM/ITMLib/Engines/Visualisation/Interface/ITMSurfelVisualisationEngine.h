// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/Camera/ITMIntrinsics.h"
#include "../../../Objects/RenderStates/ITMSurfelRenderState.h"
#include "../../../Objects/Scene/ITMSurfelScene.h"
#include "../../../Objects/Tracking/ITMTrackingState.h"
#include "../../../Objects/Views/ITMView.h"
#include "../../../Utils/ITMImageTypes.h"
#include "../../../../ORUtils/SE3Pose.h"

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
      RENDER_COLOUR,
      RENDER_CONFIDENCE,
      RENDER_FLAT,
      RENDER_LAMBERTIAN,
      RENDER_NORMAL,
      RENDER_PHONG,
    };

    //#################### DESTRUCTOR ####################
  public:
    /**
     * \brief Destroys the visualisation engine.
     */
    virtual ~ITMSurfelVisualisationEngine();

    //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
  public:
    /**
     * \brief TODO
     */
    virtual void CopyCorrespondencesToBuffers(const ITMSurfelScene<TSurfel> *scene, float *newPositions, float *oldPositions, float *correspondences) const = 0;

    /**
     * \brief TODO
     */
    virtual void CopySceneToBuffers(const ITMSurfelScene<TSurfel> *scene, float *positions, unsigned char *normals, unsigned char *colours) const = 0;

    /**
     * \brief TODO
     */
    virtual void CreateICPMaps(const ITMSurfelScene<TSurfel> *scene, const ITMSurfelRenderState *renderState, ITMTrackingState *trackingState) const = 0;

    /**
     * \brief TODO
     */
    virtual void RenderDepthImage(const ITMSurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose, const ITMSurfelRenderState *renderState,
                                  ITMFloatImage *outputImage) const = 0;

    /**
     * \brief TODO
     */
    virtual void RenderImage(const ITMSurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose, const ITMSurfelRenderState *renderState,
                             ITMUChar4Image *outputImage, RenderImageType type = RENDER_LAMBERTIAN) const = 0;

    //#################### PUBLIC MEMBER FUNCTIONS ####################
  public:
    /**
     * \brief TODO
     */
    void FindSurface(const ITMSurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
                     bool useRadii, ITMSurfelRenderState *renderState) const;

    /**
     * \brief TODO
     */
    void FindSurfaceSuper(const ITMSurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
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
    virtual void MakeIndexImage(const ITMSurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
                                int width, int height, int scaleFactor, unsigned int *surfelIndexImage, bool useRadii,
                                int *depthBuffer) const = 0;
  };
}
