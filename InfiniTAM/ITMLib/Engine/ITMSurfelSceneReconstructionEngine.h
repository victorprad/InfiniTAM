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
    /** The normal map corresponding to the live depth image. */
    ORUtils::MemoryBlock<Vector3f> *m_normalMap;

    /** The radius map corresponding to the live depth image. */
    ORUtils::MemoryBlock<float> *m_radiusMap;

    /** The vertex map corresponding to the live depth image (obtained by unprojecting the points in the depth image). */
    ORUtils::MemoryBlock<Vector3f> *m_vertexMap;

    //#################### CONSTRUCTORS ####################
  protected:
    /**
     * \brief TODO
     */
    explicit ITMSurfelSceneReconstructionEngine(const Vector2i& depthImageSize)
    {
      size_t pixelCount = depthImageSize.x * depthImageSize.y;
      m_normalMap = new ORUtils::MemoryBlock<Vector3f>(pixelCount, true, true);
      m_radiusMap = new ORUtils::MemoryBlock<float>(pixelCount, true, true);
      m_vertexMap =  new ORUtils::MemoryBlock<Vector3f>(pixelCount, true, true);
    }

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
    virtual ~ITMSurfelSceneReconstructionEngine()
    {
      delete m_normalMap;
      delete m_radiusMap;
      delete m_vertexMap;
    }

    //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
  public:
    /**
     * \brief TODO
     */
    virtual void AllocateSceneFromDepth(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const = 0;

    /**
     * \brief Updates the specified surfel-based scene by integrating depth and possibly colour information from the given view.
     *
     * \param scene         The scene to update.
     * \param view          The current view (containing the live input images from the current image source).
     * \param trackingState The current tracking state.
     */
    virtual void IntegrateIntoScene(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const = 0;

    /**
     * \brief Resets the specified surfel-based scene.
     *
     * \param scene The scene to reset.
     */
    virtual void ResetScene(ITMSurfelScene<TSurfel> *scene) const = 0;

    //#################### PRIVATE ABSTRACT MEMBER FUNCTIONS ####################
  private:
    /**
     * \brief TODO
     */
    virtual void PreprocessDepthMap(const ITMView *view) const = 0;
  };
}
