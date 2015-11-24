// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSurfelSceneReconstructionEngine_CUDA.h"

#ifdef _MSC_VER
  // Suppress some VC++ warnings that are produced when including the Thrust headers.
  #pragma warning(disable:4267)
#endif

#include <thrust/device_ptr.h>
#include <thrust/scan.h>

#ifdef _MSC_VER
  // Reenable the suppressed warnings for the rest of the translation unit.
  #pragma warning(default:4267)
#endif

#include "../../DeviceAgnostic/ITMSurfelSceneReconstructionEngine.h"

#define DEBUGGING 0

namespace ITMLib
{

//#################### CUDA KERNELS ####################

template <typename TSurfel>
__global__ void ck_add_new_surfel(int pixelCount, Matrix4f T, const unsigned int *newPointsMask, const unsigned int *newPointsPrefixSum,
                                  const Vector3f *vertexMap, const Vector4f *normalMap, const float *radiusMap, TSurfel *newSurfels)
{
  int locId = threadIdx.x + blockDim.x * blockIdx.x;
  if(locId < pixelCount)
  {
    add_new_surfel(locId, T, newPointsMask, newPointsPrefixSum, vertexMap, normalMap, radiusMap, newSurfels);
  }
}

__global__ void ck_find_corresponding_surfel(int pixelCount, const float *depthMap, const unsigned int *indexMap, unsigned int *newPointsMask)
{
  int locId = threadIdx.x + blockDim.x * blockIdx.x;
  if(locId < pixelCount)
  {
    // TEMPORARY
    find_corresponding_surfel(locId, depthMap, indexMap, newPointsMask);
  }
}

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
  const ITMPose& pose = *trackingState->pose_d;
  PreprocessDepthMap(view);
  GenerateIndexMap(scene, view, pose);
  FindCorrespondingSurfels(scene, view);
  //FuseMatchedPoints();
  AddNewSurfels(scene, pose);
  // TODO
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CUDA<TSurfel>::AddNewSurfels(ITMSurfelScene<TSurfel> *scene, const ITMPose& pose) const
{
  // Calculate the prefix sum of the new points mask.
  const unsigned int *newPointsMask = this->m_newPointsMaskMB->GetData(MEMORYDEVICE_CUDA);
  unsigned int *newPointsPrefixSum = this->m_newPointsPrefixSumMB->GetData(MEMORYDEVICE_CUDA);
  const int pixelCount = static_cast<int>(this->m_newPointsMaskMB->dataSize - 1);
  thrust::device_ptr<const unsigned int> newPointsMaskBegin(newPointsMask);
  thrust::device_ptr<unsigned int> newPointsPrefixSumBegin(newPointsPrefixSum);
  thrust::exclusive_scan(newPointsMaskBegin, newPointsMaskBegin + (pixelCount + 1), newPointsPrefixSumBegin);

#if DEBUGGING
  this->m_newPointsPrefixSumMB->UpdateHostFromDevice();
#endif

  // Add the new surfels to the scene.
  const size_t newSurfelCount = static_cast<size_t>(this->m_newPointsPrefixSumMB->GetElement(pixelCount, MEMORYDEVICE_CUDA));
  TSurfel *newSurfels = scene->AllocateSurfels(newSurfelCount);
  if(newSurfels == NULL) return;

  int threadsPerBlock = 256;
  int numBlocks = (pixelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_add_new_surfel<<<numBlocks,threadsPerBlock>>>(
    pixelCount,
    pose.GetInvM(),
    newPointsMask,
    newPointsPrefixSum,
    this->m_vertexMapMB->GetData(MEMORYDEVICE_CUDA),
    this->m_normalMapMB->GetData(MEMORYDEVICE_CUDA),
    this->m_radiusMapMB->GetData(MEMORYDEVICE_CUDA),
    newSurfels
  );

#if DEBUGGING
  scene->GetSurfels()->UpdateHostFromDevice();
#endif
}

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CUDA<TSurfel>::FindCorrespondingSurfels(const ITMSurfelScene<TSurfel> *scene, const ITMView *view) const
{
  const int pixelCount = static_cast<int>(view->depth->dataSize);

  int threadsPerBlock = 256;
  int numBlocks = (pixelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_find_corresponding_surfel<<<numBlocks,threadsPerBlock>>>(
    pixelCount,
    view->depth->GetData(MEMORYDEVICE_CUDA),
    this->m_indexMapMB->GetData(MEMORYDEVICE_CUDA),
    this->m_newPointsMaskMB->GetData(MEMORYDEVICE_CUDA)
  );

#if DEBUGGING
  this->m_newPointsMaskMB->UpdateHostFromDevice();
#endif
}

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CUDA<TSurfel>::GenerateIndexMap(const ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMPose& pose) const
{
  const int surfelCount = static_cast<int>(scene->GetSurfelCount());

  int threadsPerBlock = 256;
  int numBlocks = (surfelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_project_to_index_map<<<numBlocks,threadsPerBlock>>>(
    surfelCount,
    scene->GetSurfels()->GetData(MEMORYDEVICE_CUDA),
    pose.GetM(),
    view->calib->intrinsics_d,
    view->depth->noDims.x,
    view->depth->noDims.y,
    this->m_indexMapMB->GetData(MEMORYDEVICE_CUDA)
  );

#if DEBUGGING
  this->m_indexMapMB->UpdateHostFromDevice();
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
    this->m_vertexMapMB->GetData(MEMORYDEVICE_CUDA)
  );

#if DEBUGGING
  this->m_vertexMapMB->UpdateHostFromDevice();
#endif

  // Calculate the normal map.
  // FIXME: We don't need to store two copies of it.
  this->m_normalMapMB->SetFrom(view->depthNormal, ORUtils::MemoryBlock<Vector4f>::CUDA_TO_CUDA);

#if DEBUGGING
  this->m_normalMapMB->UpdateHostFromDevice();
#endif

  // TODO: Calculate the radius map.
}

//#################### EXPLICIT INSTANTIATIONS ####################

template class ITMSurfelSceneReconstructionEngine_CUDA<ITMSurfel>;

}
