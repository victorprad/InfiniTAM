// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#pragma once

#include "../Interface/ITMSurfelSceneReconstructionEngine.h"

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
     * \brief Constructs a CUDA-based surfel scene reconstruction engine.
     *
     * \param depthImageSize  The size of the depth images that are being fused into the scene.
     */
    explicit ITMSurfelSceneReconstructionEngine_CUDA(const Vector2i& depthImageSize);

    //#################### PRIVATE MEMBER FUNCTIONS ####################
  private:
    /** Override */
    virtual void AddNewSurfels(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const;

    /** Override */
    virtual void FindCorrespondingSurfels(const ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState,
                                          const ITMSurfelRenderState *renderState) const;

    /** Override */
    virtual void FuseMatchedPoints(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const;

    /** Override */
    virtual void MarkBadSurfels(ITMSurfelScene<TSurfel> *scene) const;

    /** Override */
    virtual void MergeSimilarSurfels(ITMSurfelScene<TSurfel> *scene, const ITMSurfelRenderState *renderState) const;

    /** Override */
    virtual void PreprocessDepthMap(const ITMView *view, const ITMSurfelSceneParams& sceneParams) const;

    /** Override */
    virtual void RemoveMarkedSurfels(ITMSurfelScene<TSurfel> *scene) const;
  };
}
