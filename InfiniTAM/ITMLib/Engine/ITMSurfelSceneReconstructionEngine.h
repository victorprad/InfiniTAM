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
    ORUtils::MemoryBlock<unsigned char> *m_newPointsMaskMB;

    /** TODO */
    ORUtils::MemoryBlock<unsigned int> *m_newPointsPrefixSumMB;

    /** The normal map corresponding to the live depth image. */
    ORUtils::MemoryBlock<Vector4f> *m_normalMapMB;

    /** The radius map corresponding to the live depth image. */
    ORUtils::MemoryBlock<float> *m_radiusMapMB;

    /** The vertex map corresponding to the live depth image (obtained by unprojecting the points in the depth image). */
    ORUtils::MemoryBlock<Vector3f> *m_vertexMapMB;

    //#################### CONSTRUCTORS ####################
  protected:
    /**
     * \brief TODO
     */
    explicit ITMSurfelSceneReconstructionEngine(const Vector2i& depthImageSize)
    {
      size_t pixelCount = depthImageSize.x * depthImageSize.y;
      m_indexMapMB = new ORUtils::MemoryBlock<unsigned int>(pixelCount * 16, true, true);
      m_newPointsMaskMB = new ORUtils::MemoryBlock<unsigned char>(pixelCount + 1, true, true);
      m_newPointsPrefixSumMB = new ORUtils::MemoryBlock<unsigned int>(pixelCount + 1, true, true);
      m_normalMapMB = new ORUtils::MemoryBlock<Vector4f>(pixelCount, true, true);
      m_radiusMapMB = new ORUtils::MemoryBlock<float>(pixelCount, true, true);
      m_vertexMapMB =  new ORUtils::MemoryBlock<Vector3f>(pixelCount, true, true);

      // Make sure that the dummy element at the end of the new points mask is initialised properly.
      m_newPointsMaskMB->GetData(MEMORYDEVICE_CPU)[pixelCount] = 0;
      m_newPointsMaskMB->UpdateDeviceFromHost();
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
      delete m_indexMapMB;
      delete m_normalMapMB;
      delete m_radiusMapMB;
      delete m_vertexMapMB;
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

    //#################### PRIVATE ABSTRACT MEMBER FUNCTIONS ####################
  private:
    /**
     * \brief TODO
     */
    virtual void AddNewSurfels(ITMSurfelScene<TSurfel> *scene) const = 0;

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
     * \brief Resets the specified surfel-based scene.
     *
     * \param scene The scene to reset.
     */
    void ResetScene(ITMSurfelScene<TSurfel> *scene) const
    {
      scene->Reset();
    }
  };
}
