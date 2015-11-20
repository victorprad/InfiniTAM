// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSurfelSceneReconstructionEngine_CPU.h"

#include "../../DeviceAgnostic/ITMSurfelSceneReconstructionEngine.h"

namespace ITMLib
{

//#################### CONSTRUCTORS ####################

template <typename TSurfel>
ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::ITMSurfelSceneReconstructionEngine_CPU(const Vector2i& depthImageSize)
: ITMSurfelSceneReconstructionEngine<TSurfel>(depthImageSize)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::AllocateSceneFromDepth(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const
{
  // TODO
}

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::IntegrateIntoScene(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const
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
void ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::AddNewSurfels(ITMSurfelScene<TSurfel> *scene, const ITMPose& pose) const
{
  // Calculate the prefix sum of the new points mask.
  const unsigned char *newPointsMask = this->m_newPointsMaskMB->GetData(MEMORYDEVICE_CPU);
  unsigned int *newPointsPrefixSum = this->m_newPointsPrefixSumMB->GetData(MEMORYDEVICE_CPU);
  const int pixelCount = static_cast<int>(this->m_newPointsMaskMB->dataSize - 1);

  newPointsPrefixSum[0] = 0;
  for(int i = 1; i <= pixelCount; ++i)
  {
    newPointsPrefixSum[i] = newPointsPrefixSum[i-1] + newPointsMask[i-1];
  }

  // Add the new surfels to the scene.
  const size_t newSurfelCount = static_cast<size_t>(newPointsPrefixSum[pixelCount]);
  TSurfel *newSurfels = scene->AllocateSurfels(newSurfelCount);
  if(newSurfels == NULL) return;

  const Matrix4f& T = pose.GetInvM();
  const Vector4f *normalMap = this->m_normalMapMB->GetData(MEMORYDEVICE_CPU);
  const float *radiusMap = this->m_radiusMapMB->GetData(MEMORYDEVICE_CPU);
  const Vector3f *vertexMap = this->m_vertexMapMB->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  //#pragma omp parallel for
#endif
  for(int locId = 0; locId < pixelCount; ++locId)
  {
    add_new_surfel(locId, T, newPointsMask, newPointsPrefixSum, vertexMap, normalMap, radiusMap, newSurfels);
  }
}

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::FindCorrespondingSurfels(const ITMSurfelScene<TSurfel> *scene, const ITMView *view) const
{
  const unsigned int *indexMap = this->m_indexMapMB->GetData(MEMORYDEVICE_CPU);
  unsigned char *newPointsMask = this->m_newPointsMaskMB->GetData(MEMORYDEVICE_CPU);
  const int pixelCount = static_cast<int>(view->depth->dataSize);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < pixelCount; ++locId)
  {
    // TEMPORARY
    find_corresponding_surfel(locId, indexMap, newPointsMask);
  }
}

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::GenerateIndexMap(const ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMPose& pose) const
{
  int depthMapHeight = view->depth->noDims.y;
  int depthMapWidth = view->depth->noDims.x;
  unsigned int *indexMap = this->m_indexMapMB->GetData(MEMORYDEVICE_CPU);
  const ITMIntrinsics& intrinsics = view->calib->intrinsics_d;
  const Matrix4f invT = pose.GetM();
  const int surfelCount = static_cast<int>(scene->GetSurfelCount());
  const TSurfel *surfels = scene->GetSurfels()->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int surfelId = 0; surfelId < surfelCount; ++surfelId)
  {
    project_to_index_map(surfelId, surfels, invT, intrinsics, depthMapWidth, depthMapHeight, indexMap);
  }
}

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::PreprocessDepthMap(const ITMView *view) const
{
  const float *depthMap = view->depth->GetData(MEMORYDEVICE_CPU);
  const ITMIntrinsics& intrinsics = view->calib->intrinsics_d;
  const int pixelCount = static_cast<int>(view->depth->dataSize);
  Vector3f *vertexMap = this->m_vertexMapMB->GetData(MEMORYDEVICE_CPU);
  const int width = view->depth->noDims.x;

  // Calculate the vertex map.
#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < pixelCount; ++locId)
  {
    calculate_vertex_position(locId, width, intrinsics, depthMap, vertexMap);
  }

  // Calculate the normal map.
  // FIXME: We don't need to store two copies of it.
  this->m_normalMapMB->SetFrom(view->depthNormal, ORUtils::MemoryBlock<Vector4f>::CPU_TO_CPU);

  // TODO: Calculate the radius map.
}

//#################### EXPLICIT INSTANTIATIONS ####################

template class ITMSurfelSceneReconstructionEngine_CPU<ITMSurfel>;

}
