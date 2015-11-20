// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSurfelVisualisationEngine_CPU.h"

#include "../../DeviceAgnostic/ITMSurfelVisualisationEngine.h"

namespace ITMLib
{

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename TSurfel>
void ITMSurfelVisualisationEngine_CPU<TSurfel>::FindSurface(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
                                                            const ITMSurfelRenderState *renderState) const
{
  float *depthBuffer = renderState->depthBuffer->GetData(MEMORYDEVICE_CPU);
  const int height = renderState->surfelIndexImage->noDims.y;
  const Matrix4f& invT = pose->GetM();
  const int surfelCount = static_cast<int>(scene->GetSurfelCount());
  unsigned long *surfelIndexImage = renderState->surfelIndexImage->GetData(MEMORYDEVICE_CPU);
  const TSurfel *surfels = scene->GetSurfels()->GetData(MEMORYDEVICE_CPU);
  const int width = renderState->surfelIndexImage->noDims.x;

#ifdef WITH_OPENMP
  //#pragma omp parallel for
#endif
  for(int surfelId = 0; surfelId < surfelCount; ++surfelId)
  {
    project_to_surfel_index_image(surfelId, surfels, invT, *intrinsics, width, height, surfelIndexImage, depthBuffer);
  }
}

template <typename TSurfel>
void ITMSurfelVisualisationEngine_CPU<TSurfel>::RenderImage(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
                                                             const ITMSurfelRenderState *renderState, ITMUChar4Image *outputImage, RenderImageType type) const
{
  // TODO
}

//#################### EXPLICIT INSTANTIATIONS ####################

template class ITMSurfelVisualisationEngine_CPU<ITMSurfel>;

}
