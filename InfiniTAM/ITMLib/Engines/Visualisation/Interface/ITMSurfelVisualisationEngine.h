// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#pragma once

#include "../Shared/ITMSurfelVisualisationEngine_Settings.h"
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
   * \brief An instance of an instantiation of a class template deriving from this one can be used to render a surfel-based 3D scene.
   */
  template <typename TSurfel>
  class ITMSurfelVisualisationEngine
  {
    //#################### ENUMERATIONS ####################
  public:
    /**
     * \brief The types of scene visualisation that the engine supports.
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
     * \brief Copies the correspondence information of all the surfels in the scene into buffers in order to support correspondence debugging using OpenGL.
     *
     * \param scene           The scene.
     * \param newPositions    The buffer into which to store the "new" positions of the surfels from their most recent merges.
     * \param oldPositions    The buffer into which to store the "old" position of the surfels from their most recent merges.
     * \param correspondences The buffer into which to store the "new" and "old" positions of the surfels for the purpose of rendering line segments between them.
     */
    virtual void CopyCorrespondencesToBuffers(const ITMSurfelScene<TSurfel> *scene, float *newPositions, float *oldPositions, float *correspondences) const = 0;

    /**
     * \brief Copies the properties of all the surfels in the scene into property-specific buffers (these can be used for rendering the scene using OpenGL).
     *
     * \param scene     The scene.
     * \param positions A buffer into which to write the surfels' positions.
     * \param normals   A buffer into which to write the surfels' normals.
     * \param colours   A buffer into which to write the surfels' colours.
     */
    virtual void CopySceneToBuffers(const ITMSurfelScene<TSurfel> *scene, float *positions, unsigned char *normals, unsigned char *colours) const = 0;

    /**
     * \brief Copies the positions and normals of the surfels in the index image into buffers that can be passed to the ICP tracker.
     *
     * \param scene           The scene.
     * \param renderState     The render state corresponding to the live camera.
     * \param trackingState   The current tracking state.
     */
    virtual void CreateICPMaps(const ITMSurfelScene<TSurfel> *scene, const ITMSurfelRenderState *renderState, ITMTrackingState *trackingState) const = 0;

    /**
     * \brief Renders a depth visualisation of the scene (as viewed from a particular camera) to an image.
     *
     * \param scene         The scene.
     * \param pose          The pose of the camera from which to render.
     * \param renderState   The render state corresponding to the camera from which to render.
     * \param outputImage   The image into which to write the result.
     */
    virtual void RenderDepthImage(const ITMSurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose, const ITMSurfelRenderState *renderState,
                                  ITMFloatImage *outputImage) const = 0;

    /**
     * \brief Renders a visualisation of the scene (as viewed from a particular camera) to an image.
     *
     * \param scene         The scene.
     * \param pose          The pose of the camera from which to render.
     * \param renderState   The render state corresponding to the camera from which to render.
     * \param outputImage   The image into which to write the result.
     * \param type          The type of visualisation to render.
     */
    virtual void RenderImage(const ITMSurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose, const ITMSurfelRenderState *renderState,
                             ITMUChar4Image *outputImage, RenderImageType type = RENDER_LAMBERTIAN) const = 0;

    //#################### PUBLIC MEMBER FUNCTIONS ####################
  public:
    /**
     * \brief Makes a non-supersampled index image in which each pixel contains the index of the surfel that projects to that point.
     *
     * \param scene                         The surfel scene.
     * \param pose                          The camera pose.
     * \param intrinsics                    The intrinsic parameters of the depth camera.
     * \param unstableSurfelRenderingMode   Whether to always/never render unstable surfels, or render them only if there's no stable alternative.
     * \param renderState                   The render state in which to store the index image.
     */
    void FindSurface(const ITMSurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
                     bool useRadii, UnstableSurfelRenderingMode unstableSurfelRenderingMode, ITMSurfelRenderState *renderState) const;

    /**
     * \brief Makes a supersampled index image in which each pixel contains the index of the surfel that projects to that point.
     *
     * \param scene                         The surfel scene.
     * \param pose                          The camera pose.
     * \param intrinsics                    The intrinsic parameters of the depth camera.
     * \param unstableSurfelRenderingMode   Whether to always/never render unstable surfels, or render them only if there's no stable alternative.
     * \param renderState                   The render state in which to store the index image.
     */
    void FindSurfaceSuper(const ITMSurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
                          UnstableSurfelRenderingMode unstableSurfelRenderingMode, ITMSurfelRenderState *renderState) const;

    //#################### PRIVATE ABSTRACT MEMBER FUNCTIONS ####################
  private:
    /**
     * \brief Gets the type of device on which the visualisation engine is operating.
     *
     * \return  The type of device on which the visualisation engine is operating.
     */
    virtual MemoryDeviceType GetMemoryType() const = 0;

    /**
     * \brief Forward projects all the surfels in the scene to make an image in which each pixel contains the index of the surfel that projected to that point.
     *
     * \param scene                         The surfel scene.
     * \param pose                          The camera pose.
     * \param intrinsics                    The intrinsic parameters of the depth camera.
     * \param width                         The width of the index image.
     * \param height                        The height of the index image.
     * \param scaleFactor                   The scale factor by which the index image is supersampled with respect to the depth image.
     * \param surfelIndexImage              The surfel index image.
     * \param useRadii                      Whether or not to render each surfel as a circle rather than a point.
     * \param unstableSurfelRenderingMode   Whether to always/never render unstable surfels, or render them only if there's no stable alternative.
     * \param depthBuffer                   The depth buffer for the index image.
     */
    virtual void MakeIndexImage(const ITMSurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
                                int width, int height, int scaleFactor, unsigned int *surfelIndexImage, bool useRadii,
                                UnstableSurfelRenderingMode unstableSurfelRenderingMode, int *depthBuffer) const = 0;
  };
}
