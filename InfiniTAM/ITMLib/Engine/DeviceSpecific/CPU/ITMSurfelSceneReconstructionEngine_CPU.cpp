// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSurfelSceneReconstructionEngine_CPU.h"

#include "../../DeviceAgnostic/ITMSurfelSceneReconstructionEngine.h"

namespace ITMLib
{

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::AllocateSceneFromDepth(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const
{
  // TODO
}

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::IntegrateIntoScene(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const
{
  // TODO
}

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::ResetScene(ITMSurfelScene<TSurfel> *scene) const
{
  // TODO
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::PreprocessDepthMap(const ITMView *view) const
{
  const float *depthMap = view->depth->GetData(MEMORYDEVICE_CPU);
  const ITMIntrinsics& intrinsics = view->calib->intrinsics_d;
  int pixelCount = static_cast<int>(view->depth->dataSize);
  Vector3f *vertexMap = m_vertexMap->GetData(MEMORYDEVICE_CPU);
  int width = view->depth->noDims.x;

  // Calculate the vertex map.
#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < pixelCount; ++locId)
  {
    calculate_vertex_position(locId, width, intrinsics, depthMap, vertexMap);
  }

  // Calculate the normal map.
  // TODO
}

//#################### EXPLICIT INSTANTIATIONS ####################

template class ITMSurfelSceneReconstructionEngine_CPU<ITMSurfel>;

}
