// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSurfelVisualisationEngine_CPU.h"

#include "../../DeviceAgnostic/ITMSurfelVisualisationEngine.h"

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
void ITMSurfelVisualisationEngine_CPU<TSurfel>::FindSurface(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
                                                            ITMSurfelRenderState *renderState) const
{
  int *depthBuffer = renderState->GetDepthBuffer()->GetData(MEMORYDEVICE_CPU);
  const int height = renderState->GetIndexImage()->noDims.y;
  const int scaleFactor = 1;
  unsigned int *surfelIndexImage = renderState->GetIndexImage()->GetData(MEMORYDEVICE_CPU);
  const int width = renderState->GetIndexImage()->noDims.x;
  MakeIndexImage(scene, pose, intrinsics, width, height, scaleFactor, surfelIndexImage, depthBuffer);
}

template <typename TSurfel>
void ITMSurfelVisualisationEngine_CPU<TSurfel>::FindSurfaceSuper(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
                                                                 ITMSurfelRenderState *renderState) const
{
  // FIXME: The 4 here shouldn't be hard-coded.
  int *depthBufferSuper = renderState->GetDepthBufferSuper()->GetData(MEMORYDEVICE_CPU);
  const int height = renderState->GetIndexImageSuper()->noDims.y;
  const int scaleFactor = 4;
  unsigned int *surfelIndexImageSuper = renderState->GetIndexImageSuper()->GetData(MEMORYDEVICE_CPU);
  const int width = renderState->GetIndexImageSuper()->noDims.x;
  MakeIndexImage(scene, pose, intrinsics, width, height, scaleFactor, surfelIndexImageSuper, depthBufferSuper);
}

template <typename TSurfel>
void ITMSurfelVisualisationEngine_CPU<TSurfel>::RenderImage(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
                                                             const ITMSurfelRenderState *renderState, ITMUChar4Image *outputImage, RenderImageType type) const
{
  // TODO
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename TSurfel>
void ITMSurfelVisualisationEngine_CPU<TSurfel>::MakeIndexImage(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
                                                               int width, int height, int scaleFactor, unsigned int *surfelIndexImage, int *depthBuffer) const
{
  const int pixelCount = width * height;

#ifdef WITH_OPENMP
  //#pragma omp parallel for
#endif
  for(int locId = 0; locId < pixelCount; ++locId)
  {
    clear_surfel_index_image(locId, surfelIndexImage, depthBuffer);
  }

  const Matrix4f& invT = pose->GetM();
  const int surfelCount = static_cast<int>(scene->GetSurfelCount());
  const TSurfel *surfels = scene->GetSurfels()->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  //#pragma omp parallel for
#endif
  for(int surfelId = 0; surfelId < surfelCount; ++surfelId)
  {
    project_to_surfel_index_image(surfelId, surfels, invT, *intrinsics, width, height, scaleFactor, surfelIndexImage, depthBuffer);
  }
}

//#################### EXPLICIT INSTANTIATIONS ####################

template class ITMSurfelVisualisationEngine_CPU<ITMSurfel>;
template class ITMSurfelVisualisationEngine_CPU<ITMSurfel_rgb>;

}
