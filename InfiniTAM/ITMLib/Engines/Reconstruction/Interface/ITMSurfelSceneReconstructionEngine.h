// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#pragma once

#include "../../../Objects/RenderStates/ITMSurfelRenderState.h"
#include "../../../Objects/Scene/ITMSurfelScene.h"
#include "../../../Objects/Tracking/ITMTrackingState.h"
#include "../../../Objects/Views/ITMView.h"

namespace ITMLib
{
  /**
   * \brief An instance of an instantiation of a class template deriving from this one can be used to make a surfel-based reconstruction of a 3D scene.
   */
  template <typename TSurfel>
  class ITMSurfelSceneReconstructionEngine
  {
    //#################### PROTECTED VARIABLES ####################
  protected:
    /** A map containing the indices of the surfels (if any) in the scene to which different points in the vertex map correspond. */
    ORUtils::MemoryBlock<unsigned int> *m_correspondenceMapMB;

    /**
     * A map representing the targets of the surfel merges that should be performed. Each element is a raster position in the index image,
     * e.g. if the merge target of the surfel specified at raster position 51 in the index image is 7, that means it should be merged into
     * the surfel specified at raster position 7 in the index image.
     */
    ORUtils::MemoryBlock<unsigned int> *m_mergeTargetMapMB;

    /** A mask whose values denote whether the corresponding points in the vertex map need to be added to the scene as new points. */
    ORUtils::MemoryBlock<unsigned short> *m_newPointsMaskMB;

    /** A prefix sum of the new points mask whose values denote the offsets in a new surfels chunk at which new points should be written. */
    ORUtils::MemoryBlock<unsigned int> *m_newPointsPrefixSumMB;

    /** The normal map corresponding to the live depth image. */
    ORUtils::MemoryBlock<Vector3f> *m_normalMapMB;

    /** The radius map corresponding to the live depth image. */
    ORUtils::MemoryBlock<float> *m_radiusMapMB;

    /** A mask whose values denote whether the corresponding surfels in the scene should be removed. */
    ORUtils::MemoryBlock<unsigned int> *m_surfelRemovalMaskMB;

    /** The current timestamp (i.e. frame number). */
    int m_timestamp;

    /**
     * The vertex map corresponding to the live depth image (obtained by back-projecting the points in the depth image).
     * The w component of each vertex is set to 1 if the corresponding depth pixel was valid, and -1 otherwise.
     */
    ORUtils::MemoryBlock<Vector4f> *m_vertexMapMB;

    //#################### CONSTRUCTORS ####################
  protected:
    /**
     * \brief Constructs a surfel scene reconstruction engine.
     *
     * \param depthImageSize  The size of the depth images that are being fused into the scene.
     */
    explicit ITMSurfelSceneReconstructionEngine(const Vector2i& depthImageSize);

    //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
  private:
    // Deliberately private and unimplemented.
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
     * \brief Adds surfels to the scene for any points in the live 3D depth image that do not correspond to an existing surfel.
     *
     * \param scene           The scene.
     * \param view            The current view (containing the live input images from the current image source).
     * \param trackingState   The current tracking state.
     */
    virtual void AddNewSurfels(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const = 0;

    /**
     * \brief Finds the indices of the surfels (if any) in the scene to which different points in the live 3D depth image map correspond.
     *
     * \param scene           The scene.
     * \param view            The current view (containing the live input images from the current image source).
     * \param trackingState   The current tracking state.
     * \param renderState     The render state corresponding to the camera from which the scene is being viewed.
     */
    virtual void FindCorrespondingSurfels(const ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState,
                                          const ITMSurfelRenderState *renderState) const = 0;

    /**
     * \brief Fuses points in the live point cloud into the surfels in the scene with which they have been matched.
     *
     * \param scene           The scene.
     * \param view            The current view (containing the live input images from the current image source).
     * \param trackingState   The current tracking state.
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
     * \brief Calculates the position, normal and radius of the surfel that would be constructed for each point in the live 3D depth image.
     *
     * \param view          The current view (containing the live input images from the current image source).
     * \param sceneParams   The parameters associated with the surfel scene.
     */
    virtual void PreprocessDepthMap(const ITMView *view, const ITMSurfelSceneParams& sceneParams) const = 0;

    /**
     * \brief Removes from the scene any surfels that have been marked by previous stages of the pipeline.
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
     * \param renderState   The render state corresponding to the camera from which the scene is being viewed.
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
