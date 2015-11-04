// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSurfelSceneReconstructionEngine_CUDA.h"

#include "../../DeviceAgnostic/ITMSurfelSceneReconstructionEngine.h"

#define DEBUGGING 1

namespace ITMLib
{

//#################### CUDA KERNELS ####################

__global__ void ck_calculate_vertex_map(int pixelCount, int width, ITMIntrinsics intrinsics, const float *depthMap, Vector3f *vertexMap)
{
  int locId = threadIdx.x + blockDim.x * blockIdx.x;
  if(locId < pixelCount)
  {
    calculate_vertex_position(locId, width, intrinsics, depthMap, vertexMap);
  }
}

//#################### CONSTRUCTORS ####################

template <typename TSurfel>
ITMSurfelSceneReconstructionEngine_CUDA<TSurfel>::ITMSurfelSceneReconstructionEngine_CUDA(const Vector2i& depthImageSize)
: ITMSurfelSceneReconstructionEngine<TSurfel>(depthImageSize)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CUDA<TSurfel>::AllocateSceneFromDepth(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const
{
  // TODO
}

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CUDA<TSurfel>::IntegrateIntoScene(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const
{
  // TEMPORARY
  PreprocessDepthMap(view);

  // TODO
}

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CUDA<TSurfel>::ResetScene(ITMSurfelScene<TSurfel> *scene) const
{
  // TODO
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CUDA<TSurfel>::PreprocessDepthMap(const ITMView *view) const
{
  const int pixelCount = static_cast<int>(view->depth->dataSize);

  // Calculate the vertex map.
  int threadsPerBlock = 256;
  int numBlocks = (pixelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_calculate_vertex_map<<<numBlocks,threadsPerBlock>>>(
    pixelCount,
    view->depth->noDims.x,
    view->calib->intrinsics_d,
    view->depth->GetData(MEMORYDEVICE_CUDA),
    m_vertexMap->GetData(MEMORYDEVICE_CUDA)
  );

#if DEBUGGING
  m_vertexMap->UpdateHostFromDevice();
#endif

  // Calculate the normal map.
  // FIXME: We don't need to store two copies of it.
  m_normalMap->SetFrom(view->depthNormal, ORUtils::MemoryBlock<Vector4f>::CUDA_TO_CUDA);

#if DEBUGGING
  m_normalMap->UpdateHostFromDevice();
#endif

  // TODO
}

//#################### EXPLICIT INSTANTIATIONS ####################

template class ITMSurfelSceneReconstructionEngine_CUDA<ITMSurfel>;

}
