// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

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
                                                        bool useRadii, UnstableSurfelRenderingMode unstableSurfelRenderingMode, ITMSurfelRenderState *renderState) const
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
    unstableSurfelRenderingMode,
    renderState->GetDepthBuffer()->GetData(memoryType)
  );
}

template <typename TSurfel>
void ITMSurfelVisualisationEngine<TSurfel>::FindSurfaceSuper(const ITMSurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
                                                             UnstableSurfelRenderingMode unstableSurfelRenderingMode, ITMSurfelRenderState *renderState) const
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
    unstableSurfelRenderingMode,
    renderState->GetDepthBufferSuper()->GetData(memoryType)
  );
}

}
