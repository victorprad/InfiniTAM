// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#include "ITMSurfelSceneReconstructionEngine_CPU.h"

#include "../Shared/ITMSurfelSceneReconstructionEngine_Shared.h"

namespace ITMLib
{

//#################### CONSTRUCTORS ####################

template <typename TSurfel>
ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::ITMSurfelSceneReconstructionEngine_CPU(const Vector2i& depthImageSize)
: ITMSurfelSceneReconstructionEngine<TSurfel>(depthImageSize)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::AddNewSurfels(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const
{
  // Calculate the prefix sum of the new points mask.
  const unsigned short *newPointsMask = this->m_newPointsMaskMB->GetData(MEMORYDEVICE_CPU);
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

  const Vector4u *colourMap = view->rgb->GetData(MEMORYDEVICE_CPU);
  const Matrix4f& depthToRGB = view->calib.trafo_rgb_to_depth.calib_inv;
  const Vector3f *normalMap = this->m_normalMapMB->GetData(MEMORYDEVICE_CPU);
  const Vector4f& projParamsRGB = view->calib.intrinsics_rgb.projectionParamsSimple.all;
  const float *radiusMap = this->m_radiusMapMB->GetData(MEMORYDEVICE_CPU);
  const ITMSurfelSceneParams& sceneParams = scene->GetParams();
  const Matrix4f T = trackingState->pose_d->GetInvM();
  const Vector4f *vertexMap = this->m_vertexMapMB->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < pixelCount; ++locId)
  {
    add_new_surfel(
      locId, T, this->m_timestamp, newPointsMask, newPointsPrefixSum, vertexMap, normalMap, radiusMap, colourMap,
      view->depth->noDims.x, view->depth->noDims.y, view->rgb->noDims.x, view->rgb->noDims.y,
      depthToRGB, projParamsRGB, sceneParams.useGaussianSampleConfidence, sceneParams.gaussianConfidenceSigma,
      sceneParams.maxSurfelRadius, newSurfels
    );
  }
}

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::FindCorrespondingSurfels(const ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState,
                                                                               const ITMSurfelRenderState *renderState) const
{
  unsigned int *correspondenceMap = this->m_correspondenceMapMB->GetData(MEMORYDEVICE_CPU);
  const float *depthMap = view->depth->GetData(MEMORYDEVICE_CPU);
  const int depthMapWidth = view->depth->noDims.x;
  const unsigned int *indexImageSuper = renderState->GetIndexImageSuper()->GetData(MEMORYDEVICE_CPU);
  const Matrix4f& invT = trackingState->pose_d->GetM();
  unsigned short *newPointsMask = this->m_newPointsMaskMB->GetData(MEMORYDEVICE_CPU);
  const Vector3f *normalMap = this->m_normalMapMB->GetData(MEMORYDEVICE_CPU);
  const int pixelCount = static_cast<int>(view->depth->dataSize);
  const ITMSurfelSceneParams& sceneParams = scene->GetParams();
  const TSurfel *surfels = scene->GetSurfels()->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < pixelCount; ++locId)
  {
    find_corresponding_surfel(locId, invT, depthMap, depthMapWidth, normalMap, indexImageSuper, sceneParams.supersamplingFactor, surfels, correspondenceMap, newPointsMask);
  }
}

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::FuseMatchedPoints(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const
{
  const Vector4u *colourMap = view->rgb->GetData(MEMORYDEVICE_CPU);
  const int colourMapHeight = view->rgb->noDims.y;
  const int colourMapWidth = view->rgb->noDims.x;
  const int depthMapHeight = view->depth->noDims.y;
  const int depthMapWidth = view->depth->noDims.x;
  const unsigned int *correspondenceMap = this->m_correspondenceMapMB->GetData(MEMORYDEVICE_CPU);
  const Matrix4f& depthToRGB = view->calib.trafo_rgb_to_depth.calib_inv;
  const Vector3f *normalMap = this->m_normalMapMB->GetData(MEMORYDEVICE_CPU);
  const int pixelCount = static_cast<int>(view->depth->dataSize);
  const Vector4f& projParamsRGB = view->calib.intrinsics_rgb.projectionParamsSimple.all;
  const float *radiusMap = this->m_radiusMapMB->GetData(MEMORYDEVICE_CPU);
  const ITMSurfelSceneParams& sceneParams = scene->GetParams();
  TSurfel *surfels = scene->GetSurfels()->GetData(MEMORYDEVICE_CPU);
  const Matrix4f T = trackingState->pose_d->GetInvM();
  const Vector4f *vertexMap = this->m_vertexMapMB->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < pixelCount; ++locId)
  {
    fuse_matched_point(
      locId, correspondenceMap, T, this->m_timestamp, vertexMap, normalMap, radiusMap, colourMap, depthMapWidth, depthMapHeight, colourMapWidth, colourMapHeight,
      depthToRGB, projParamsRGB, sceneParams.deltaRadius, sceneParams.useGaussianSampleConfidence, sceneParams.gaussianConfidenceSigma, sceneParams.maxSurfelRadius,
      surfels
    );
  }
}

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::MarkBadSurfels(ITMSurfelScene<TSurfel> *scene) const
{
  const int surfelCount = static_cast<int>(scene->GetSurfelCount());

  // If the scene is empty, early out.
  if(surfelCount == 0) return;

  const ITMSurfelSceneParams& sceneParams = scene->GetParams();
  unsigned int *surfelRemovalMask = this->m_surfelRemovalMaskMB->GetData(MEMORYDEVICE_CPU);
  const TSurfel *surfels = scene->GetSurfels()->GetData(MEMORYDEVICE_CPU);

  // Clear the surfel removal mask.
#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int surfelId = 0; surfelId < surfelCount; ++surfelId)
  {
    clear_removal_mask_entry(surfelId, surfelRemovalMask);
  }

  // Mark long-term unstable surfels for removal.
#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int surfelId = 0; surfelId < surfelCount; ++surfelId)
  {
    mark_for_removal_if_unstable(
      surfelId,
      surfels,
      this->m_timestamp,
      sceneParams.stableSurfelConfidence,
      sceneParams.unstableSurfelPeriod,
      surfelRemovalMask
    );
  }
}

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::MergeSimilarSurfels(ITMSurfelScene<TSurfel> *scene, const ITMSurfelRenderState *renderState) const
{
  const unsigned int *correspondenceMap = this->m_correspondenceMapMB->GetData(MEMORYDEVICE_CPU);
  const unsigned int *indexImage = renderState->GetIndexImage()->GetData(MEMORYDEVICE_CPU);
  const int indexImageHeight = renderState->GetIndexImage()->noDims.y;
  const int indexImageWidth = renderState->GetIndexImage()->noDims.x;
  unsigned int *mergeTargetMap = this->m_mergeTargetMapMB->GetData(MEMORYDEVICE_CPU);
  const int pixelCount = static_cast<int>(renderState->GetIndexImage()->dataSize);
  const ITMSurfelSceneParams& sceneParams = scene->GetParams();
  TSurfel *surfels = scene->GetSurfels()->GetData(MEMORYDEVICE_CPU);
  unsigned int *surfelRemovalMask = this->m_surfelRemovalMaskMB->GetData(MEMORYDEVICE_CPU);

  // Clear the merge target map.
#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < pixelCount; ++locId)
  {
    clear_merge_target(locId, mergeTargetMap);
  }

  // Find pairs of surfels that can be merged.
#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < pixelCount; ++locId)
  {
    find_mergeable_surfel(
      locId, indexImage, indexImageWidth, indexImageHeight, correspondenceMap, surfels,
      sceneParams.stableSurfelConfidence, sceneParams.maxMergeDist, sceneParams.maxMergeAngle,
      sceneParams.minRadiusOverlapFactor, mergeTargetMap
    );
  }

  // Prevent any merge chains.
#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < pixelCount; ++locId)
  {
    prevent_merge_chain(locId, mergeTargetMap);
  }

  // Merge the relevant surfels.
#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < pixelCount; ++locId)
  {
    perform_surfel_merge(locId, mergeTargetMap, surfels, surfelRemovalMask, indexImage, sceneParams.maxSurfelRadius);
  }
}

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::PreprocessDepthMap(const ITMView *view, const ITMSurfelSceneParams& sceneParams) const
{
  const float *depthMap = view->depth->GetData(MEMORYDEVICE_CPU);
  const int height = view->depth->noDims.y;
  const ITMIntrinsics& intrinsics = view->calib.intrinsics_d;
  Vector3f *normalMap = this->m_normalMapMB->GetData(MEMORYDEVICE_CPU);
  const int pixelCount = static_cast<int>(view->depth->dataSize);
  float *radiusMap = this->m_radiusMapMB->GetData(MEMORYDEVICE_CPU);
  Vector4f *vertexMap = this->m_vertexMapMB->GetData(MEMORYDEVICE_CPU);
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
#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < pixelCount; ++locId)
  {
    calculate_normal(locId, vertexMap, width, height, normalMap);
  }

  // Calculate the radius map.
#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < pixelCount; ++locId)
  {
    calculate_radius(locId, depthMap, normalMap, intrinsics, radiusMap);
  }
}

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::RemoveMarkedSurfels(ITMSurfelScene<TSurfel> *scene) const
{
  // Remove marked surfels from the scene.
  // TODO: This is is currently unimplemented on CPU. It's not worth implementing it at the moment,
  // because we're going to need to change the scene representation to something better anyway.
}

}
