// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#include "ITMSurfelVisualisationEngine_CPU.h"

#include <stdexcept>

#include "../Shared/ITMSurfelVisualisationEngine_Shared.h"

namespace ITMLib
{

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename TSurfel>
void ITMSurfelVisualisationEngine_CPU<TSurfel>::CopyCorrespondencesToBuffers(const ITMSurfelScene<TSurfel> *scene, float *newPositions, float *oldPositions, float *correspondences) const
{
  const int surfelCount = static_cast<int>(scene->GetSurfelCount());
  const TSurfel *surfels = scene->GetSurfels()->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int surfelId = 0; surfelId < surfelCount; ++surfelId)
  {
    copy_correspondences_to_buffers(surfelId, surfels, newPositions, oldPositions, correspondences);
  }
}

template <typename TSurfel>
void ITMSurfelVisualisationEngine_CPU<TSurfel>::CopySceneToBuffers(const ITMSurfelScene<TSurfel> *scene, float *positions, unsigned char *normals, unsigned char *colours) const
{
  const int surfelCount = static_cast<int>(scene->GetSurfelCount());
  const TSurfel *surfels = scene->GetSurfels()->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int surfelId = 0; surfelId < surfelCount; ++surfelId)
  {
    copy_surfel_to_buffers(surfelId, surfels, positions, normals, colours);
  }
}

template <typename TSurfel>
void ITMSurfelVisualisationEngine_CPU<TSurfel>::CreateICPMaps(const ITMSurfelScene<TSurfel> *scene, const ITMSurfelRenderState *renderState, ITMTrackingState *trackingState) const
{
  const Matrix4f& invT = trackingState->pose_d->GetM();
  Vector4f *normalsMap = trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CPU);
  const int pixelCount = static_cast<int>(renderState->GetIndexImage()->dataSize);
  Vector4f *pointsMap = trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CPU);
  const ITMSurfelSceneParams& sceneParams = scene->GetParams();
  const unsigned int *surfelIndexImage = renderState->GetIndexImage()->GetData(MEMORYDEVICE_CPU);
  const TSurfel *surfels = scene->GetSurfels()->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < pixelCount; ++locId)
  {
    copy_surfel_data_to_icp_maps(locId, surfels, surfelIndexImage, invT, sceneParams.trackingSurfelMaxDepth, sceneParams.trackingSurfelMinConfidence, pointsMap, normalsMap);
  }
}

template <typename TSurfel>
void ITMSurfelVisualisationEngine_CPU<TSurfel>::RenderDepthImage(const ITMSurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose,
                                                                 const ITMSurfelRenderState *renderState, ITMFloatImage *outputImage) const
{
  const Vector3f cameraPosition = pose->GetT();
  float *outputImagePtr = outputImage->GetData(MEMORYDEVICE_CPU);
  const int pixelCount = static_cast<int>(outputImage->dataSize);
  const unsigned int *surfelIndexImagePtr = renderState->GetIndexImage()->GetData(MEMORYDEVICE_CPU);
  const TSurfel *surfels = scene->GetSurfels()->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < pixelCount; ++locId)
  {
    shade_pixel_depth(locId, surfelIndexImagePtr, surfels, cameraPosition, outputImagePtr);
  }
}

template <typename TSurfel>
void ITMSurfelVisualisationEngine_CPU<TSurfel>::RenderImage(const ITMSurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose, const ITMSurfelRenderState *renderState,
                                                            ITMUChar4Image *outputImage, RenderImageType type) const
{
  // Prevent colour rendering if the surfels don't store colour information.
  if(type == Base::RENDER_COLOUR && !TSurfel::hasColourInformation) type = Base::RENDER_LAMBERTIAN;

  Vector4u *outputImagePtr = outputImage->GetData(MEMORYDEVICE_CPU);
  const int pixelCount = static_cast<int>(outputImage->dataSize);
  const ITMSurfelSceneParams& sceneParams = scene->GetParams();
  const unsigned int *surfelIndexImagePtr = renderState->GetIndexImage()->GetData(MEMORYDEVICE_CPU);
  const TSurfel *surfels = scene->GetSurfels()->GetData(MEMORYDEVICE_CPU);

  switch(type)
  {
    case Base::RENDER_COLOUR:
    {
#ifdef WITH_OPENMP
      #pragma omp parallel for
#endif
      for(int locId = 0; locId < pixelCount; ++locId)
      {
        shade_pixel_colour(locId, surfelIndexImagePtr, surfels, outputImagePtr);
      }
      break;
    }
    case Base::RENDER_CONFIDENCE:
    {
#ifdef WITH_OPENMP
      #pragma omp parallel for
#endif
      for(int locId = 0; locId < pixelCount; ++locId)
      {
        shade_pixel_confidence(locId, surfelIndexImagePtr, surfels, sceneParams.stableSurfelConfidence, outputImagePtr);
      }
      break;
    }
    case Base::RENDER_FLAT:
    case Base::RENDER_LAMBERTIAN:
    case Base::RENDER_PHONG:
    {
      SurfelLightingType lightingType = SLT_LAMBERTIAN;
      if(type == Base::RENDER_FLAT) lightingType = SLT_FLAT;
      else if(type == Base::RENDER_PHONG) lightingType = SLT_PHONG;

      const Vector3f lightPos = Vector3f(0.0f, -10.0f, -10.0f);
      const Vector3f viewerPos = Vector3f(pose->GetInvM().getColumn(3));

#ifdef WITH_OPENMP
      #pragma omp parallel for
#endif
      for(int locId = 0; locId < pixelCount; ++locId)
      {
        shade_pixel_grey(locId, surfelIndexImagePtr, surfels, lightPos, viewerPos, lightingType, outputImagePtr);
      }
      break;
    }
    case Base::RENDER_NORMAL:
    {
#ifdef WITH_OPENMP
      #pragma omp parallel for
#endif
      for(int locId = 0; locId < pixelCount; ++locId)
      {
        shade_pixel_normal(locId, surfelIndexImagePtr, surfels, outputImagePtr);
      }
      break;
    }
    default:
    {
      // This should never happen.
      throw std::runtime_error("Unsupported surfel visualisation type");
    }
  }
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename TSurfel>
MemoryDeviceType ITMSurfelVisualisationEngine_CPU<TSurfel>::GetMemoryType() const
{
  return MEMORYDEVICE_CPU;
}

template <typename TSurfel>
void ITMSurfelVisualisationEngine_CPU<TSurfel>::MakeIndexImage(const ITMSurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
                                                               int width, int height, int scaleFactor, unsigned int *surfelIndexImage, bool useRadii,
                                                               UnstableSurfelRenderingMode unstableSurfelRenderingMode, int *depthBuffer) const
{
  const int pixelCount = width * height;

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < pixelCount; ++locId)
  {
    clear_surfel_index_image(locId, surfelIndexImage, depthBuffer);
  }

  const Matrix4f& invT = pose->GetM();
  const ITMSurfelSceneParams& sceneParams = scene->GetParams();
  const int surfelCount = static_cast<int>(scene->GetSurfelCount());
  const TSurfel *surfels = scene->GetSurfels()->GetData(MEMORYDEVICE_CPU);

  // Note: This is deliberately not parallelised (it would be slower due to the synchronisation needed).
  for(int surfelId = 0; surfelId < surfelCount; ++surfelId)
  {
    update_depth_buffer_for_surfel(
      surfelId, surfels, invT, *intrinsics, width, height, scaleFactor, useRadii, unstableSurfelRenderingMode,
      sceneParams.stableSurfelConfidence, sceneParams.unstableSurfelZOffset, depthBuffer
    );
  }

  // Note: This is deliberately not parallelised (it would be slower due to the synchronisation needed).
  for(int surfelId = 0; surfelId < surfelCount; ++surfelId)
  {
    update_index_image_for_surfel(
      surfelId, surfels, invT, *intrinsics, width, height, scaleFactor, depthBuffer, useRadii, unstableSurfelRenderingMode,
      sceneParams.stableSurfelConfidence, sceneParams.unstableSurfelZOffset, surfelIndexImage
    );
  }
}

}
