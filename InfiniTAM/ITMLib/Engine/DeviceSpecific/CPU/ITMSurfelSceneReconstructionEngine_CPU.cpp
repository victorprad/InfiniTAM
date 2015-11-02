// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSurfelSceneReconstructionEngine_CPU.h"

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

//#################### EXPLICIT INSTANTIATIONS ####################

template class ITMSurfelSceneReconstructionEngine_CPU<ITMSurfel>;

}
