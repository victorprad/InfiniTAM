// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDenseSurfelMapper.h"

namespace ITMLib
{

//#################### CONSTRUCTORS ####################

template <typename TSurfel>
ITMDenseSurfelMapper<TSurfel>::ITMDenseSurfelMapper()
{
  // TODO
}

//#################### DESTRUCTOR ####################

template <typename TSurfel>
ITMDenseSurfelMapper<TSurfel>::~ITMDenseSurfelMapper()
{
  // TODO
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename TSurfel>
void ITMDenseSurfelMapper<TSurfel>::ProcessFrame(const ITMView *view, const ITMTrackingState *trackingState, ITMSurfelScene<TSurfel> *scene, ITMRenderState *liveRenderState) const
{
  // TODO
}

//#################### EXPLICIT INSTANTIATIONS ####################

template class ITMDenseSurfelMapper<ITMSurfel>;

}
