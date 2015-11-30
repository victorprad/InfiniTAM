// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../ORUtils/PlatformIndependence.h"
#include "../../Objects/ITMIntrinsics.h"

namespace ITMLib
{

//#################### HELPERS ####################

/**
 * \brief TODO
 *
 * \param invT  A transformation from global coordinates to pose coordinates.
 * \param p     The point whose depth we want to calculate.
 */
_CPU_AND_GPU_CODE_
inline float calculate_depth_from_pose(const Matrix4f& invT, const Vector3f& p)
{
  Vector4f vg(p.x, p.y, p.z, 1.0f);
  Vector4f v = invT * vg;
  return v.z;
}

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline Vector3f transform_point(const Matrix4f& T, const Vector3f& p)
{
  Vector4f v(p.x, p.y, p.z, 1.0f);
  return (T * v).toVector3();
}

//#################### MAIN FUNCTIONS ####################

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void add_new_surfel(int locId, const Matrix4f& T, const unsigned short *newPointsMask, const unsigned int *newPointsPrefixSum,
                           const Vector3f *vertexMap, const Vector4f *normalMap, const float *radiusMap, const Vector4u *colourMap,
                           int timestamp, TSurfel *newSurfels, const TSurfel *surfels, const unsigned int *correspondenceMap)
{
  if(newPointsMask[locId])
  {
    TSurfel surfel;
    surfel.position = transform_point(T, vertexMap[locId]);
    surfel.normal = normalMap[locId].toVector3();
    surfel.radius = radiusMap[locId];
    surfel.confidence = 1.0f;                     // TEMPORARY
    surfel.timestamp = timestamp;

    // Store a colour if the surfel type can support it.
    SurfelColourManipulator<TSurfel::hasColourInformation>::write(surfel, colourMap[locId].toVector3());  // TEMPORARY

#if DEBUG_CORRESPONDENCES
    // Store the position of the corresponding surfel (if any).
    int correspondingSurfelIndex = correspondenceMap[locId] - 1;
    surfel.correspondingSurfelPosition = correspondingSurfelIndex >= 0 ? surfels[correspondingSurfelIndex].position : surfel.position;
#endif

    newSurfels[newPointsPrefixSum[locId]] = surfel;
  }
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
inline void clear_removal_mask(int surfelId, unsigned int *surfelRemovalMask)
{
  surfelRemovalMask[surfelId] = 0;
}

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void find_corresponding_surfel(int locId, const Matrix4f& invT, const float *depthMap, int depthMapWidth, const unsigned int *indexMap, const TSurfel *surfels,
                                      unsigned int *correspondenceMap, unsigned short *newPointsMask)
{
  // If the depth pixel is invalid, early out.
  float depth = depthMap[locId];
  if(fabs(depth + 1) <= 0.0001f)
  {
    correspondenceMap[locId] = 0;
    newPointsMask[locId] = 0;
    return;
  }

  // Otherwise, find corresponding surfels in the scene and pick the best one (if any).
  int bestSurfelIndex = -1;
  float bestSurfelConfidence = 0.0f;
  int ux = locId % depthMapWidth, uy = locId / depthMapWidth;
  for(int dy = 0; dy < 4; ++dy)
  {
    for(int dx = 0; dx < 4; ++dx)
    {
      int x = ux * 4 + dx;
      int y = uy * 4 + dy;
      int surfelIndex = indexMap[y * depthMapWidth * 4 + x] - 1;
      if(surfelIndex >= 0)
      {
        // TODO: Make this slightly more sophisticated, as per the paper.
        TSurfel surfel = surfels[surfelIndex];
        float surfelDepth = calculate_depth_from_pose(invT, surfel.position);

        const float deltaDepth = 0.01f;
        if(surfel.confidence > bestSurfelConfidence && fabs(surfelDepth - depth) <= deltaDepth)
        {
          bestSurfelIndex = surfelIndex;
          bestSurfelConfidence = surfel.confidence;
        }
      }
    }
  }

  // Record any corresponding surfel found, together with a flag indicating whether or not we need to add a new surfel.
  correspondenceMap[locId] = bestSurfelIndex >= 0 ? bestSurfelIndex + 1 : 0;
  newPointsMask[locId] = bestSurfelIndex >= 0 ? 0 : 1;

#if DEBUG_CORRESPONDENCES
  newPointsMask[locId] = 1;
#endif
}

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void fuse_matched_point(int locId, const unsigned int *correspondenceMap, const Matrix4f& T, const Vector3f *vertexMap,
                               const Vector4f *normalMap, const float *radiusMap, const Vector4u *colourMap, int timestamp,
                               TSurfel *surfels)
{
  // TEMPORARY
  const float alpha = 1.0f;

  int surfelIndex = correspondenceMap[locId] - 1;
  if(surfelIndex >= 0)
  {
    TSurfel surfel = surfels[surfelIndex];

    const float newConfidence = surfel.confidence + alpha;
    surfel.position = (surfel.confidence * surfel.position + alpha * transform_point(T, vertexMap[locId])) / newConfidence;

    // TODO: Normal, radius, etc.

    Vector3u oldColour = SurfelColourManipulator<TSurfel::hasColourInformation>::read(surfel);
    Vector3u newColour = colourMap[locId].toVector3();
    Vector3u colour = ((surfel.confidence * oldColour.toFloat() + alpha * newColour.toFloat()) / newConfidence).toUChar();
    SurfelColourManipulator<TSurfel::hasColourInformation>::write(surfel, colour);

    surfel.confidence = newConfidence;
    surfel.timestamp = timestamp;

    surfels[surfelIndex] = surfel;
  }
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
  Vector3f p = surfels[surfelId].position;
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
    // Write the surfel ID + 1 into the index map.
    indexMap[y * indexMapWidth + x] = static_cast<unsigned int>(surfelId + 1);
  }
}

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void mark_for_removal_if_unstable(int surfelId, const TSurfel *surfels, int timestamp, unsigned int *surfelRemovalMask)
{
  // TEMPORARY
  const float stableConfidence = 10.0f;
  TSurfel surfel = surfels[surfelId];
  if(surfel.confidence < stableConfidence && timestamp - surfel.timestamp > 5)
  {
    surfelRemovalMask[surfelId] = 1;
  }
}

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void reset_index_map_pixel(int indexLocId, unsigned int *indexMap)
{
  indexMap[indexLocId] = 0;
}

}
