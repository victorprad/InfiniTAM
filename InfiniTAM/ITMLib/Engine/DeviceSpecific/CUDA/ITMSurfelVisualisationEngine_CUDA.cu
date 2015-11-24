// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSurfelVisualisationEngine_CUDA.h"

#include "../../DeviceAgnostic/ITMSurfelVisualisationEngine.h"

namespace ITMLib
{

//#################### CUDA KERNELS ####################

template <typename TSurfel>
__global__ void ck_copy_scene_to_buffers(int surfelCount, const TSurfel *surfels, float *positions)
{
  int surfelId = threadIdx.x + blockDim.x * blockIdx.x;
  if(surfelId < surfelCount)
  {
    copy_surfel_to_buffers(surfelId, surfels, positions);
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename TSurfel>
void ITMSurfelVisualisationEngine_CUDA<TSurfel>::CopySceneToBuffers(const ITMSurfelScene<TSurfel> *scene, float *positions) const
{
  const int surfelCount = static_cast<int>(scene->GetSurfelCount());

  int threadsPerBlock = 256;
  int numBlocks = (surfelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_copy_scene_to_buffers<<<numBlocks,threadsPerBlock>>>(
    surfelCount,
    scene->GetSurfels()->GetData(MEMORYDEVICE_CUDA),
    positions
  );
}

template <typename TSurfel>
void ITMSurfelVisualisationEngine_CUDA<TSurfel>::FindSurface(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
                                                             const ITMSurfelRenderState *renderState) const
{
  // TODO
}

template <typename TSurfel>
void ITMSurfelVisualisationEngine_CUDA<TSurfel>::RenderImage(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
                                                             const ITMSurfelRenderState *renderState, ITMUChar4Image *outputImage, RenderImageType type) const
{
  // TODO
}

//#################### EXPLICIT INSTANTIATIONS ####################

template class ITMSurfelVisualisationEngine_CUDA<ITMSurfel>;

}
