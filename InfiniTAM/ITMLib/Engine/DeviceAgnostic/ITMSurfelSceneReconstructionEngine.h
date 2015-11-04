// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../ORUtils/PlatformIndependence.h"
#include "../../Objects/ITMIntrinsics.h"

namespace ITMLib
{

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void calculate_vertex_position(int locId, int width, const ITMIntrinsics& intrinsics, const float *depthMap, Vector3f *vertexMap)
{
  /*
  v(~u~) = D(~u~) K^{-1} (~u~^T,1)^T
         = D(~u~) (fx 0 px)^{-1} (ux) = D(~u~) ((ux - px) / fx)
                  (0 fy py)      (uy)          ((uy - py) / fy)
                  (0  0  1)      ( 1)          (             1)
  */
  int ux = locId % width, uy = locId / width;
  vertexMap[locId] = depthMap[locId] * Vector3f(
    (ux - intrinsics.projectionParamsSimple.px) / intrinsics.projectionParamsSimple.fx,
    (uy - intrinsics.projectionParamsSimple.py) / intrinsics.projectionParamsSimple.fy,
    1
  );
}

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void project_to_index_map(int surfelId, const TSurfel *surfels, const ITMPose& pose, const ITMIntrinsics& intrinsics, unsigned int *indexMap)
{
  /*
  TODO
  */
  // TODO
}

}
