// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSurfelVisualisationEngine_CUDA.h"

#include "../../DeviceAgnostic/ITMSurfelVisualisationEngine.h"

namespace ITMLib
{

//#################### CUDA KERNELS ####################

__global__ void ck_clear_surfel_index_image(int pixelCount, unsigned int *surfelIndexImage, int *depthBuffer)
{
  int locId = threadIdx.x + blockDim.x * blockIdx.x;
  if(locId < pixelCount)
  {
    clear_surfel_index_image(locId, surfelIndexImage, depthBuffer);
  }
}

#if DEBUG_CORRESPONDENCES
template <typename TSurfel>
__global__ void ck_copy_correspondences_to_buffer(int surfelCount, const TSurfel *surfels, float *correspondences)
{
  int surfelId = threadIdx.x + blockDim.x * blockIdx.x;
  if(surfelId < surfelCount)
  {
    copy_correspondences_to_buffer(surfelId, surfels, correspondences);
  }
}
#endif

template <typename TSurfel>
__global__ void ck_copy_scene_to_buffers(int surfelCount, const TSurfel *surfels, float *positions, unsigned char *normals, unsigned char *colours)
{
  int surfelId = threadIdx.x + blockDim.x * blockIdx.x;
  if(surfelId < surfelCount)
  {
    copy_surfel_to_buffers(surfelId, surfels, positions, normals, colours);
  }
}

template <typename TSurfel>
__global__ void ck_project_to_surfel_index_image(int surfelCount, const TSurfel *surfels, Matrix4f invT, ITMIntrinsics intrinsics, int indexImageWidth, int indexImageHeight,
                                                 int scaleFactor, unsigned int *surfelIndexImage, int *depthBuffer)
{
  int surfelId = threadIdx.x + blockDim.x * blockIdx.x;
  if(surfelId < surfelCount)
  {
    project_to_surfel_index_image(surfelId, surfels, invT, intrinsics, indexImageWidth, indexImageHeight, scaleFactor, surfelIndexImage, depthBuffer);
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

#if DEBUG_CORRESPONDENCES
template <typename TSurfel>
void ITMSurfelVisualisationEngine_CUDA<TSurfel>::CopyCorrespondencesToBuffer(const ITMSurfelScene<TSurfel> *scene, float *correspondences) const
{
  const int surfelCount = static_cast<int>(scene->GetSurfelCount());

  int threadsPerBlock = 256;
  int numBlocks = (surfelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_copy_correspondences_to_buffer<<<numBlocks,threadsPerBlock>>>(
    surfelCount,
    scene->GetSurfels()->GetData(MEMORYDEVICE_CUDA),
    correspondences
  );
}
#endif

template <typename TSurfel>
void ITMSurfelVisualisationEngine_CUDA<TSurfel>::CopySceneToBuffers(const ITMSurfelScene<TSurfel> *scene, float *positions, unsigned char *normals, unsigned char *colours) const
{
  const int surfelCount = static_cast<int>(scene->GetSurfelCount());

  int threadsPerBlock = 256;
  int numBlocks = (surfelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_copy_scene_to_buffers<<<numBlocks,threadsPerBlock>>>(
    surfelCount,
    scene->GetSurfels()->GetData(MEMORYDEVICE_CUDA),
    positions,
    normals,
    colours
  );
}

template <typename TSurfel>
void ITMSurfelVisualisationEngine_CUDA<TSurfel>::RenderImage(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
                                                             const ITMSurfelRenderState *renderState, ITMUChar4Image *outputImage, RenderImageType type) const
{
  // TODO
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename TSurfel>
MemoryDeviceType ITMSurfelVisualisationEngine_CUDA<TSurfel>::GetMemoryType() const
{
  return MEMORYDEVICE_CUDA;
}

template <typename TSurfel>
void ITMSurfelVisualisationEngine_CUDA<TSurfel>::MakeIndexImage(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
                                                                int width, int height, int scaleFactor, unsigned int *surfelIndexImage, int *depthBuffer) const
{
  const int pixelCount = width * height;

  int threadsPerBlock = 256;
  int numBlocks = (pixelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_clear_surfel_index_image<<<numBlocks,threadsPerBlock>>>(
    pixelCount,
    surfelIndexImage,
    depthBuffer
  );

  const int surfelCount = static_cast<int>(scene->GetSurfelCount());
  numBlocks = (surfelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_project_to_surfel_index_image<<<numBlocks,threadsPerBlock>>>(
    surfelCount,
    scene->GetSurfels()->GetData(MEMORYDEVICE_CUDA),
    pose->GetM(),
    *intrinsics,
    width,
    height,
    scaleFactor,
    surfelIndexImage,
    depthBuffer
  );
}

//#################### EXPLICIT INSTANTIATIONS ####################

template class ITMSurfelVisualisationEngine_CUDA<ITMSurfel>;
template class ITMSurfelVisualisationEngine_CUDA<ITMSurfel_rgb>;

}
