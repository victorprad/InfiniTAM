// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSurfelVisualisationEngine.h"

namespace ITMLib
{

template <typename TSurfel>
void ITMSurfelVisualisationEngine<TSurfel>::FindSurface(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
                                                        ITMSurfelRenderState *renderState) const
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
    renderState->GetDepthBuffer()->GetData(memoryType)
  );
}

template <typename TSurfel>
void ITMSurfelVisualisationEngine<TSurfel>::FindSurfaceSuper(const ITMSurfelScene<TSurfel> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
                                                             ITMSurfelRenderState *renderState) const
{
  MemoryDeviceType memoryType = GetMemoryType();
  MakeIndexImage(
    scene,
    pose,
    intrinsics,
    renderState->GetIndexImageSuper()->noDims.x,
    renderState->GetIndexImageSuper()->noDims.y,
    renderState->GetSuperScaleFactor(),
    renderState->GetIndexImageSuper()->GetData(memoryType),
    renderState->GetDepthBufferSuper()->GetData(memoryType)
  );
}

}
