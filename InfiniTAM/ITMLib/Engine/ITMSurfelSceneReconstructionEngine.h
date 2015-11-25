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
    //#################### PROTECTED VARIABLES ####################
  protected:
    /** TODO */
    ORUtils::MemoryBlock<unsigned int> *m_indexMapMB;

    /** A mask whose values denote whether the corresponding points in the vertex map need to be added to the scene as new points. */
    ORUtils::MemoryBlock<unsigned int> *m_newPointsMaskMB;

    /** TODO */
    ORUtils::MemoryBlock<unsigned int> *m_newPointsPrefixSumMB;

    /** The normal map corresponding to the live depth image. */
    ORUtils::MemoryBlock<Vector4f> *m_normalMapMB;

    /** The radius map corresponding to the live depth image. */
    ORUtils::MemoryBlock<float> *m_radiusMapMB;

    /** TODO */
    int m_timestamp;

    /** The vertex map corresponding to the live depth image (obtained by unprojecting the points in the depth image). */
    ORUtils::MemoryBlock<Vector3f> *m_vertexMapMB;

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
    virtual void FindCorrespondingSurfels(const ITMSurfelScene<TSurfel> *scene, const ITMView *view) const = 0;

    /**
     * \brief TODO
     */
    virtual void GenerateIndexMap(const ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMPose& pose) const = 0;

    /**
     * \brief TODO
     */
    virtual void PreprocessDepthMap(const ITMView *view) const = 0;

    //#################### PUBLIC MEMBER FUNCTIONS ####################
  public:
    /**
     * \brief Updates the specified surfel-based scene by integrating depth and possibly colour information from the given view.
     *
     * \param scene         The scene to update.
     * \param view          The current view (containing the live input images from the current image source).
     * \param trackingState The current tracking state.
     */
    void IntegrateIntoScene(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState);

    /**
     * \brief Resets the specified surfel-based scene.
     *
     * \param scene The scene to reset.
     */
    void ResetScene(ITMSurfelScene<TSurfel> *scene) const;
  };
}
