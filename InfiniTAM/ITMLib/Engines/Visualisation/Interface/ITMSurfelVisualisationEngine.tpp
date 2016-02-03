// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSurfelVisualisationEngine.h"

namespace ITMLib
{

//#################### DESTRUCTOR ####################

template <typename TSurfel>
ITMSurfelVisualisationEngine<TSurfel>::~ITMSurfelVisualisationEngine()
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename TSurfel>
void ITMSurfelVisualisationEngine<TSurfel>::FindSurface(const ITMSurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
                                                        bool useRadii, ITMSurfelRenderState *renderState) const
{
  MemoryDeviceType memoryType = GetMemoryType();
  MakeIndexImage(
    scene,
    pose,
    intrinsics,
    renderState->GetIndexImage()->noDims.x,
    renderState->GetIndexImage()->noDims.y,
    1,
    renderState->GetIndexImage()->GetData(memoryType),
    useRadii,
    renderState->GetDepthBuffer()->GetData(memoryType)
  );
}

template <typename TSurfel>
void ITMSurfelVisualisationEngine<TSurfel>::FindSurfaceSuper(const ITMSurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
                                                             ITMSurfelRenderState *renderState) const
{
  MemoryDeviceType memoryType = GetMemoryType();
  MakeIndexImage(
    scene,
    pose,
    intrinsics,
    renderState->GetIndexImageSuper()->noDims.x,
    renderState->GetIndexImageSuper()->noDims.y,
    scene->GetParams().supersamplingFactor,
    renderState->GetIndexImageSuper()->GetData(memoryType),
    false,
    renderState->GetDepthBufferSuper()->GetData(memoryType)
  );
}

}
