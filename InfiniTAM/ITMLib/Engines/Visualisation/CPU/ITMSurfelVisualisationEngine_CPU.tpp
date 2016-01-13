// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSurfelVisualisationEngine_CPU.h"

#include "../Shared/ITMSurfelVisualisationEngine_Shared.h"

namespace ITMLib
{

//#################### PUBLIC MEMBER FUNCTIONS ####################

#if DEBUG_CORRESPONDENCES
template <typename TSurfel>
void ITMSurfelVisualisationEngine_CPU<TSurfel>::CopyCorrespondencesToBuffer(const ITMSurfelScene<TSurfel> *scene, float *correspondences) const
{
  const int surfelCount = static_cast<int>(scene->GetSurfelCount());
  const TSurfel *surfels = scene->GetSurfels()->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int surfelId = 0; surfelId < surfelCount; ++surfelId)
  {
    copy_correspondences_to_buffer(surfelId, surfels, correspondences);
  }
}
#endif

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
  const unsigned int *surfelIndexImage = renderState->GetIndexImage()->GetData(MEMORYDEVICE_CPU);
  const TSurfel *surfels = scene->GetSurfels()->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < pixelCount; ++locId)
  {
    copy_surfel_data_to_icp_maps(locId, surfels, surfelIndexImage, invT, pointsMap, normalsMap);
  }
}

template <typename TSurfel>
void ITMSurfelVisualisationEngine_CPU<TSurfel>::RenderDepthImage(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose,
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
void ITMSurfelVisualisationEngine_CPU<TSurfel>::RenderImage(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMSurfelRenderState *renderState,
                                                            ITMUChar4Image *outputImage, RenderImageType type) const
{
  // Prevent colour rendering if the surfels don't store colour information.
  if(type == Base::RENDER_COLOUR && !TSurfel::hasColourInformation) type = Base::RENDER_LAMBERTIAN;

  Vector4u *outputImagePtr = outputImage->GetData(MEMORYDEVICE_CPU);
  const int pixelCount = static_cast<int>(outputImage->dataSize);
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
    case Base::RENDER_LAMBERTIAN:
    {
      const Vector3f lightSource = -Vector3f(pose->GetInvM().getColumn(2));

#ifdef WITH_OPENMP
      #pragma omp parallel for
#endif
      for(int locId = 0; locId < pixelCount; ++locId)
      {
        shade_pixel_grey(locId, surfelIndexImagePtr, surfels, lightSource, outputImagePtr);
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
      // TODO
      break;
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
void ITMSurfelVisualisationEngine_CPU<TSurfel>::MakeIndexImage(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
                                                               int width, int height, int scaleFactor, unsigned int *surfelIndexImage, int *depthBuffer) const
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
  const int surfelCount = static_cast<int>(scene->GetSurfelCount());
  const TSurfel *surfels = scene->GetSurfels()->GetData(MEMORYDEVICE_CPU);

  // Note: This is deliberately not parallelised (it would be slower due to the synchronisation needed).
  for(int surfelId = 0; surfelId < surfelCount; ++surfelId)
  {
    update_depth_buffer_for_surfel(surfelId, surfels, invT, *intrinsics, width, height, scaleFactor, depthBuffer);
  }

  // Note: This is deliberately not parallelised (it would be slower due to the synchronisation needed).
  for(int surfelId = 0; surfelId < surfelCount; ++surfelId)
  {
    update_index_image_for_surfel(surfelId, surfels, invT, *intrinsics, width, height, scaleFactor, depthBuffer, surfelIndexImage);
  }
}

}
