// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSurfelSceneReconstructionEngine_CUDA.h"

#ifdef _MSC_VER
  // Suppress some VC++ warnings that are produced when including the Thrust headers.
  #pragma warning(disable:4267)
#endif

#include <thrust/device_ptr.h>
#include <thrust/scan.h>
#include <thrust/sort.h>

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
__global__ void ck_add_new_surfel(int pixelCount, Matrix4f T, const unsigned short *newPointsMask, const unsigned int *newPointsPrefixSum,
                                  const Vector3f *vertexMap, const Vector4f *normalMap, const float *radiusMap, const Vector4u *colourMap,
                                  int timestamp, TSurfel *newSurfels, const TSurfel *surfels, const unsigned int *correspondenceMap)
{
  int locId = threadIdx.x + blockDim.x * blockIdx.x;
  if(locId < pixelCount)
  {
    add_new_surfel(locId, T, newPointsMask, newPointsPrefixSum, vertexMap, normalMap, radiusMap, colourMap, timestamp, newSurfels, surfels, correspondenceMap);
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

__global__ void ck_clear_removal_mask(int surfelCount, unsigned int *surfelRemovalMask)
{
  int surfelId = threadIdx.x + blockDim.x * blockIdx.x;
  if(surfelId < surfelCount)
  {
    clear_removal_mask(surfelId, surfelRemovalMask);
  }
}

template <typename TSurfel>
__global__ void ck_find_corresponding_surfel(int pixelCount, const float *depthMap, int depthMapWidth, const unsigned int *indexMap, const TSurfel *surfels,
                                             unsigned int *correspondenceMap, unsigned short *newPointsMask)
{
  int locId = threadIdx.x + blockDim.x * blockIdx.x;
  if(locId < pixelCount)
  {
    find_corresponding_surfel(locId, depthMap, depthMapWidth, indexMap, surfels, correspondenceMap, newPointsMask);
  }
}

template <typename TSurfel>
__global__ void ck_fuse_matched_point(int pixelCount, const unsigned int *correspondenceMap, Matrix4f T, const Vector3f *vertexMap, const Vector4f *normalMap,
                                      const float *radiusMap, const Vector4u *colourMap, int timestamp, TSurfel *surfels)
{
  int locId = threadIdx.x + blockDim.x * blockIdx.x;
  if(locId < pixelCount)
  {
    fuse_matched_point(locId, correspondenceMap, T, vertexMap, normalMap, radiusMap, colourMap, timestamp, surfels);
  }
}

template <typename TSurfel>
__global__ void ck_mark_for_removal_if_unstable(int surfelCount, const TSurfel *surfels, int timestamp, unsigned int *surfelRemovalMask)
{
  int surfelId = threadIdx.x + blockDim.x * blockIdx.x;
  if(surfelId < surfelCount)
  {
    mark_for_removal_if_unstable(surfelId, surfels, timestamp, surfelRemovalMask);
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

__global__ void ck_reset_index_map(int indexPixelCount, unsigned int *indexMap)
{
  int indexLocId = threadIdx.x + blockDim.x * blockIdx.x;
  if(indexLocId < indexPixelCount)
  {
    reset_index_map_pixel(indexLocId, indexMap);
  }
}

//#################### CONSTRUCTORS ####################

template <typename TSurfel>
ITMSurfelSceneReconstructionEngine_CUDA<TSurfel>::ITMSurfelSceneReconstructionEngine_CUDA(const Vector2i& depthImageSize)
: ITMSurfelSceneReconstructionEngine<TSurfel>(depthImageSize)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CUDA<TSurfel>::AddNewSurfels(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const
{
  // Calculate the prefix sum of the new points mask.
  const unsigned short *newPointsMask = this->m_newPointsMaskMB->GetData(MEMORYDEVICE_CUDA);
  unsigned int *newPointsPrefixSum = this->m_newPointsPrefixSumMB->GetData(MEMORYDEVICE_CUDA);
  const int pixelCount = static_cast<int>(this->m_newPointsMaskMB->dataSize - 1);
  thrust::device_ptr<const unsigned short> newPointsMaskBegin(newPointsMask);
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
    trackingState->pose_d->GetInvM(),
    newPointsMask,
    newPointsPrefixSum,
    this->m_vertexMapMB->GetData(MEMORYDEVICE_CUDA),
    this->m_normalMapMB->GetData(MEMORYDEVICE_CUDA),
    this->m_radiusMapMB->GetData(MEMORYDEVICE_CUDA),
    view->rgb->GetData(MEMORYDEVICE_CUDA),
    this->m_timestamp,
    newSurfels,
    scene->GetSurfels()->GetData(MEMORYDEVICE_CUDA),
    this->m_correspondenceMapMB->GetData(MEMORYDEVICE_CUDA)
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
    view->depth->noDims.x,
    this->m_indexMapMB->GetData(MEMORYDEVICE_CUDA),
    scene->GetSurfels()->GetData(MEMORYDEVICE_CUDA),
    this->m_correspondenceMapMB->GetData(MEMORYDEVICE_CUDA),
    this->m_newPointsMaskMB->GetData(MEMORYDEVICE_CUDA)
  );

#if DEBUGGING
  this->m_correspondenceMapMB->UpdateHostFromDevice();
  this->m_newPointsMaskMB->UpdateHostFromDevice();
#endif
}

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CUDA<TSurfel>::FuseMatchedPoints(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const
{
  const int pixelCount = static_cast<int>(view->depth->dataSize);

  int threadsPerBlock = 256;
  int numBlocks = (pixelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_fuse_matched_point<<<numBlocks,threadsPerBlock>>>(
    pixelCount,
    this->m_correspondenceMapMB->GetData(MEMORYDEVICE_CUDA),
    trackingState->pose_d->GetInvM(),
    this->m_vertexMapMB->GetData(MEMORYDEVICE_CUDA),
    this->m_normalMapMB->GetData(MEMORYDEVICE_CUDA),
    this->m_radiusMapMB->GetData(MEMORYDEVICE_CUDA),
    view->rgb->GetData(MEMORYDEVICE_CUDA),
    this->m_timestamp,
    scene->GetSurfels()->GetData(MEMORYDEVICE_CUDA)
  );
}

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CUDA<TSurfel>::GenerateIndexMap(const ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMPose& pose) const
{
  const int indexPixelCount = static_cast<int>(this->m_indexMapMB->dataSize);
  unsigned int *indexMap = this->m_indexMapMB->GetData(MEMORYDEVICE_CUDA);
  int threadsPerBlock = 256;
  int numBlocks = (indexPixelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_reset_index_map<<<numBlocks,threadsPerBlock>>>(
    indexPixelCount,
    indexMap
  );

  const int surfelCount = static_cast<int>(scene->GetSurfelCount());
  if(surfelCount > 0)
  {
    numBlocks = (surfelCount + threadsPerBlock - 1) / threadsPerBlock;
    ck_project_to_index_map<<<numBlocks,threadsPerBlock>>>(
      surfelCount,
      scene->GetSurfels()->GetData(MEMORYDEVICE_CUDA),
      pose.GetM(),
      view->calib->intrinsics_d,
      view->depth->noDims.x,
      view->depth->noDims.y,
      indexMap
    );
  }

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

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CUDA<TSurfel>::RemoveBadSurfels(ITMSurfelScene<TSurfel> *scene) const
{
  const int surfelCount = static_cast<int>(scene->GetSurfelCount());
  unsigned int *surfelRemovalMask = this->m_surfelRemovalMaskMB->GetData(MEMORYDEVICE_CUDA);
  TSurfel *surfels = scene->GetSurfels()->GetData(MEMORYDEVICE_CUDA);

  int threadsPerBlock = 256;
  int numBlocks = (surfelCount + threadsPerBlock - 1) / threadsPerBlock;

  // Clear the surfel removal mask.
  ck_clear_removal_mask<<<numBlocks,threadsPerBlock>>>(
    surfelCount,
    surfelRemovalMask
  );

  // Mark long-term unstable surfels for removal.
  ck_mark_for_removal_if_unstable<<<numBlocks,threadsPerBlock>>>(
    surfelCount,
    surfels,
    this->m_timestamp,
    surfelRemovalMask
  );

#if DEBUGGING
  scene->GetSurfels()->UpdateHostFromDevice();
  this->m_surfelRemovalMaskMB->UpdateHostFromDevice();
#endif

  // Remove marked surfels from the scene.
  thrust::device_ptr<unsigned int> surfelRemovalMaskBegin(surfelRemovalMask);
  thrust::device_ptr<unsigned int> surfelRemovalMaskEnd = surfelRemovalMaskBegin + surfelCount;
  thrust::device_ptr<TSurfel> surfelsBegin(surfels);
  thrust::sort_by_key(surfelRemovalMaskBegin, surfelRemovalMaskEnd, surfelsBegin);
  scene->DeallocateRemovedSurfels(thrust::reduce(surfelRemovalMaskBegin, surfelRemovalMaskEnd));
}

//#################### EXPLICIT INSTANTIATIONS ####################

template class ITMSurfelSceneReconstructionEngine_CUDA<ITMSurfel>;
template class ITMSurfelSceneReconstructionEngine_CUDA<ITMSurfel_rgb>;

}
