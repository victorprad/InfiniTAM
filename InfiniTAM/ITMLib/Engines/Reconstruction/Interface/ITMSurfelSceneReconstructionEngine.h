// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/RenderStates/ITMSurfelRenderState.h"
#include "../../../Objects/Scene/ITMSurfelScene.h"
#include "../../../Objects/Tracking/ITMTrackingState.h"
#include "../../../Objects/Views/ITMView.h"

namespace ITMLib
{
  /**
   * \brief An instance of an instantiation of this class template can be used to make a surfel-based reconstruction of a 3D scene.
   */
  template <typename TSurfel>
  class ITMSurfelSceneReconstructionEngine
  {
    //#################### PROTECTED VARIABLES ####################
  protected:
    /** TODO */
    ORUtils::MemoryBlock<unsigned int> *m_correspondenceMapMB;

    /** TODO */
    ORUtils::MemoryBlock<unsigned int> *m_mergeMapMB;

    /** A mask whose values denote whether the corresponding points in the vertex map need to be added to the scene as new points. */
    ORUtils::MemoryBlock<unsigned short> *m_newPointsMaskMB;

    /** TODO */
    ORUtils::MemoryBlock<unsigned int> *m_newPointsPrefixSumMB;

    /** The normal map corresponding to the live depth image. */
    ORUtils::MemoryBlock<Vector3f> *m_normalMapMB;

    /** The radius map corresponding to the live depth image. */
    ORUtils::MemoryBlock<float> *m_radiusMapMB;

    /** TODO */
    ORUtils::MemoryBlock<unsigned int> *m_surfelRemovalMaskMB;

    /** TODO */
    int m_timestamp;

    /**
     * The vertex map corresponding to the live depth image (obtained by unprojecting the points in the depth image).
     * The w component of each vertex is set to 1 if the corresponding depth pixel was valid, and -1 otherwise.
     */
    ORUtils::MemoryBlock<Vector4f> *m_vertexMapMB;

    //#################### CONSTRUCTORS ####################
  protected:
    /**
     * \brief TODO
     */
    explicit ITMSurfelSceneReconstructionEngine(const Vector2i& depthImageSize);

    //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
  private:
    // Deliberately private and unimplemented
    ITMSurfelSceneReconstructionEngine(const ITMSurfelSceneReconstructionEngine&);
    ITMSurfelSceneReconstructionEngine& operator=(const ITMSurfelSceneReconstructionEngine&);

    //#################### DESTRUCTOR ####################
  public:
    /**
     * \brief Destroys the reconstruction engine.
     */
    virtual ~ITMSurfelSceneReconstructionEngine();

    //#################### PRIVATE ABSTRACT MEMBER FUNCTIONS ####################
  private:
    /**
     * \brief TODO
     */
    virtual void AddNewSurfels(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const = 0;

    /**
     * \brief TODO
     */
    virtual void FindCorrespondingSurfels(const ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState,
                                          const ITMSurfelRenderState *renderState) const = 0;

    /**
     * \brief TODO
     */
    virtual void FuseMatchedPoints(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const = 0;

    /**
     * \brief Marks surfels that should be removed from the scene.
     *
     * \param scene The surfel scene.
     */
    virtual void MarkBadSurfels(ITMSurfelScene<TSurfel> *scene) const = 0;

    /**
     * \brief Merges together surfels in the current index image that have sufficiently similar positions and normals, and whose radii overlap.
     *
     * Surfels that need to be removed as a result of the merging process will be removed alongside any bad surfels that already been marked.
     *
     * \param scene       The surfel scene.
     * \param renderState The render state corresponding to the camera from which the scene is being viewed.
     */
    virtual void MergeSimilarSurfels(ITMSurfelScene<TSurfel> *scene, const ITMSurfelRenderState *renderState) const = 0;

    /**
     * \brief TODO
     */
    virtual void PreprocessDepthMap(const ITMView *view) const = 0;

    /**
     * \brief Removes any surfels that have been marked from the scene.
     *
     * \param scene The surfel scene.
     */
    virtual void RemoveMarkedSurfels(ITMSurfelScene<TSurfel> *scene) const = 0;

    //#################### PUBLIC MEMBER FUNCTIONS ####################
  public:
    /**
     * \brief Updates the specified surfel-based scene by integrating depth and possibly colour information from the given view.
     *
     * \param scene         The scene to update.
     * \param view          The current view (containing the live input images from the current image source).
     * \param trackingState The current tracking state.
     * \param renderState   The current render state.
     */
    void IntegrateIntoScene(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState, const ITMSurfelRenderState *renderState);

    /**
     * \brief Resets the specified surfel-based scene.
     *
     * \param scene The scene to reset.
     */
    void ResetScene(ITMSurfelScene<TSurfel> *scene) const;
  };
}
