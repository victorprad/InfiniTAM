// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSurfelSceneReconstructionEngine_CUDA.h"

#include "../../DeviceAgnostic/ITMSurfelSceneReconstructionEngine.h"

#define DEBUGGING 1

namespace ITMLib
{

//#################### CUDA KERNELS ####################

template <typename TSurfel>
__global__ void ck_project_to_index_map(int surfelCount, const TSurfel *surfels, Matrix4f invT, ITMIntrinsics intrinsics, int depthMapWidth, int depthMapHeight,
                                        unsigned int *indexMap)
{
  int surfelId = threadIdx.x + blockDim.x * blockIdx.x;
  if(surfelId < surfelCount)
  {
    project_to_index_map(surfelId, surfels, invT, intrinsics, depthMapWidth, depthMapHeight, indexMap);
  }
}

__global__ void ck_calculate_vertex_position(int pixelCount, int width, ITMIntrinsics intrinsics, const float *depthMap, Vector3f *vertexMap)
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
  //GenerateIndexMap(scene, view, *trackingState->pose_d);

  // TODO
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CUDA<TSurfel>::GenerateIndexMap(const ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMPose& pose) const
{
  const int surfelCount = static_cast<int>(scene->GetSurfelCount());

  int threadsPerBlock = 256;
  int numBlocks = (surfelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_project_to_index_map<<<numBlocks,threadsPerBlock>>>(
    surfelCount,
    scene->GetSurfels()->GetData(MEMORYDEVICE_CUDA),
    pose.GetInvM(),
    view->calib->intrinsics_d,
    view->depth->noDims.x,
    view->depth->noDims.y,
    m_indexMap->GetData(MEMORYDEVICE_CUDA)
  );

#if DEBUGGING
  m_indexMap->UpdateHostFromDevice();
#endif
}

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CUDA<TSurfel>::PreprocessDepthMap(const ITMView *view) const
{
  const int pixelCount = static_cast<int>(view->depth->dataSize);

  // Calculate the vertex map.
  int threadsPerBlock = 256;
  int numBlocks = (pixelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_calculate_vertex_position<<<numBlocks,threadsPerBlock>>>(
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

  // TODO: Calculate the radius map.
}

//#################### EXPLICIT INSTANTIATIONS ####################

template class ITMSurfelSceneReconstructionEngine_CUDA<ITMSurfel>;

}
