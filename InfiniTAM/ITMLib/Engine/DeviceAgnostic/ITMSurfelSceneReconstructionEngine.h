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
         = D(~u~) (fx 0 px) (ux) = D(~u~) (fx.ux + px)
                  (0 fy py) (uy)          (fy.uy + py)
                  (0  0  1) ( 1)          (         1)
  */
  int ux = locId % width, uy = locId / width;
  vertexMap[locId] = depthMap[locId] * Vector3f(
    intrinsics.projectionParamsSimple.fx * ux + intrinsics.projectionParamsSimple.px,
    intrinsics.projectionParamsSimple.fy * uy + intrinsics.projectionParamsSimple.py,
    1
  );
}

}
