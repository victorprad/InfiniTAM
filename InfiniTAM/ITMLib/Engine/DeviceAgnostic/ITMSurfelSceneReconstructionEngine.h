// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../ORUtils/PlatformIndependence.h"
#include "../../Objects/ITMIntrinsics.h"

namespace ITMLib
{

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void add_new_surfel(int locId, const unsigned char *newPointsMask, const unsigned int *newPointsPrefixSum,
                           const Vector3f *vertexMap, const Vector4f *normalMap, const float *radiusMap, TSurfel *newSurfels)
{
  // TODO
}

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
_CPU_AND_GPU_CODE_
inline void find_corresponding_surfel(int locId, const unsigned int *indexMap, unsigned char *newPointsMask)
{
  // TEMPORARY
  newPointsMask[locId] = 1;
}

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void project_to_index_map(int surfelId, const TSurfel *surfels, const Matrix4f& invT, const ITMIntrinsics& intrinsics,
                                 int depthMapWidth, int depthMapHeight, unsigned int *indexMap)
{
  // Convert the surfel point into the coordinates of the current frame using v_i = T_i^{-1} v_i^g.
  const Vector3f& p = surfels[surfelId].position;
  Vector4f vg(p.x, p.y, p.z, 1.0f);
  Vector4f v = invT * vg;

  // Project the point onto the image plane of the current frame.
  float ux = intrinsics.projectionParamsSimple.fx * v.x / v.z + intrinsics.projectionParamsSimple.px;
  float uy = intrinsics.projectionParamsSimple.fy * v.y / v.z + intrinsics.projectionParamsSimple.py;

  // Convert the projected point into index map coordinates.
  // FIXME: The 4s here shouldn't be hard-coded.
  int x = static_cast<int>(ux * 4 + 0.5f);
  int y = static_cast<int>(uy * 4 + 0.5f);

  int indexMapWidth = depthMapWidth * 4;
  int indexMapHeight = depthMapHeight * 4;
  if(0 <= x && x < indexMapWidth && 0 <= y && y < indexMapHeight)
  {
    // Write the surfel ID into the index map.
    indexMap[y * indexMapWidth + x] = static_cast<unsigned int>(surfelId);
  }
}

}
