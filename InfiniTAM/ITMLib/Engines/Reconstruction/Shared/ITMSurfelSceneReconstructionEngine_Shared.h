// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/Camera/ITMIntrinsics.h"
#include "../../../Objects/Scene/ITMRepresentationAccess.h"

namespace ITMLib
{

//#################### HELPERS ####################

_CPU_AND_GPU_CODE_ Vector3f transform_normal(const Matrix4f& T, const Vector3f& n);
_CPU_AND_GPU_CODE_ Vector3f transform_point(const Matrix4f& T, const Vector3f& p);

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
inline float calculate_gaussian_sample_confidence(int locId, int width, int height, float sigma)
{
  const int x = locId % width, y = locId / width;
  const float halfW = width / 2.0f, halfH = height / 2.0f;
  const float dx = abs(x - halfW), dy = abs(y - halfH);
  const float gamma = sqrtf((dx * dx + dy * dy) / (halfW * halfW + halfH * halfH));
  return expf(-gamma*gamma) / (2*sigma*sigma);
}

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline Vector3u compute_colour(const Vector3f& v, const Matrix4f& depthToRGB, const Vector4f& projParamsRGB,
                               const Vector4u *colourMap, int colourMapWidth, int colourMapHeight)
{
  Vector3f cv = transform_point(depthToRGB, v);
  int x = static_cast<int>(projParamsRGB.x * cv.x / cv.z + projParamsRGB.z + 0.5f);
  int y = static_cast<int>(projParamsRGB.y * cv.y / cv.z + projParamsRGB.w + 0.5f);
  Vector3u colour((uchar)0);
  if(x >= 0 && x < colourMapWidth && y >= 0 && y < colourMapHeight)
  {
    colour = colourMap[y * colourMapWidth + x].toVector3();
  }
  return colour;
}

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline Vector3f transform_normal(const Matrix4f& T, const Vector3f& n)
{
  Vector4f v(n.x, n.y, n.z, 0.0f);
  return (T * v).toVector3();
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

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline bool try_read_vertex(int x, int y, const Vector4f *vertexMap, int width, int height, Vector3f& result)
{
  if(x >= 0 && x < width && y >= 0 && y < height)
  {
    Vector4f v = vertexMap[y * width + x];
    if(v.w > 0.0f)
    {
      result = v.toVector3();
      return true;
    }
  }
  return false;
}

//#################### MAIN FUNCTIONS ####################

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void add_new_surfel(int locId, const Matrix4f& T, int timestamp, const unsigned short *newPointsMask, const unsigned int *newPointsPrefixSum,
                           const Vector4f *vertexMap, const Vector3f *normalMap, const float *radiusMap, const Vector4u *colourMap,
                           int depthMapWidth, int depthMapHeight, int colourMapWidth, int colourMapHeight, const Matrix4f& depthToRGB,
                           const Vector4f& projParamsRGB, bool useGaussianSampleConfidence, float gaussianConfidenceSigma, TSurfel *newSurfels,
                           const TSurfel *surfels, const unsigned int *correspondenceMap)
{
  if(newPointsMask[locId])
  {
    const Vector3f v = vertexMap[locId].toVector3();

    TSurfel surfel;
    surfel.position = transform_point(T, v);
    surfel.normal = transform_normal(T, normalMap[locId]);
    surfel.radius = radiusMap[locId];
    surfel.confidence = useGaussianSampleConfidence ? calculate_gaussian_sample_confidence(locId, depthMapWidth, depthMapHeight, gaussianConfidenceSigma) : 1.0f;
    surfel.timestamp = timestamp;

    // Store a colour if the surfel type can support it.
    Vector3u colour = compute_colour(v, depthToRGB, projParamsRGB, colourMap, colourMapWidth, colourMapHeight);
    SurfelColourManipulator<TSurfel::hasColourInformation>::write(surfel, colour);

    newSurfels[newPointsPrefixSum[locId]] = surfel;
  }
}

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void calculate_normal(int locId, const Vector4f *vertexMap, int width, int height, Vector3f *normalMap)
{
  // FIXME: This is a bit of a quick hack at the moment - it can be improved in due course.
  int x = locId % width, y = locId / width;

  Vector3f n(0.0f);

  bool ok = true;
  Vector3f xm, xp, ym, yp;
  ok = ok && try_read_vertex(x - 2, y, vertexMap, width, height, xm);
  ok = ok && try_read_vertex(x + 2, y, vertexMap, width, height, xp);
  ok = ok && try_read_vertex(x, y - 2, vertexMap, width, height, ym);
  ok = ok && try_read_vertex(x, y + 2, vertexMap, width, height, yp);

  if(ok)
  {
    Vector3f xdiff = xp - xm, ydiff = yp - ym;
    n = ORUtils::cross(ydiff, xdiff);

    float len = length(n);
    if(len > 0.0f) n /= len;
  }

  normalMap[locId] = n;
}

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void calculate_radius(int locId, const float *depthMap, const Vector3f *normalMap, const ITMIntrinsics& intrinsics,
                             float *radiusMap)
{
  float r = 0.0f;
  Vector3f n = normalMap[locId];

  if(length(n) > 0.0f)
  {
    // FIXME: This is currently a total hack.
    float d = depthMap[locId];
    if(d > 10.0f) d = 10.0f;
    r = sqrt(2.0f) * d;
  }

  radiusMap[locId] = r;
}

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void calculate_vertex_position(int locId, int width, const ITMIntrinsics& intrinsics, const float *depthMap, Vector4f *vertexMap)
{
  /*
  v(~u~) = D(~u~) K^{-1} (~u~^T,1)^T
         = D(~u~) (fx 0 px)^{-1} (ux) = D(~u~) ((ux - px) / fx)
                  (0 fy py)      (uy)          ((uy - py) / fy)
                  (0  0  1)      ( 1)          (             1)
  */
  int ux = locId % width, uy = locId / width;
  Vector4f value(0.0f, 0.0f, 0.0f, -1.0f);
  const float depth = depthMap[locId];
  const float EPSILON = 1e-3f;
  if(fabs(depth + 1) > EPSILON)
  {
    value = Vector4f(
      depth * (ux - intrinsics.projectionParamsSimple.px) / intrinsics.projectionParamsSimple.fx,
      depth * (uy - intrinsics.projectionParamsSimple.py) / intrinsics.projectionParamsSimple.fy,
      depth,
      1.0f
    );
  }
  vertexMap[locId] = value;
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
inline void find_corresponding_surfel(int locId, const Matrix4f& invT, const float *depthMap, int depthMapWidth, const Vector3f *normalMap, const unsigned int *indexMap,
                                      int supersamplingFactor, const TSurfel *surfels, unsigned int *correspondenceMap, unsigned short *newPointsMask)
{
  // If the depth pixel or normal is invalid, early out.
  const float EPSILON = 1e-3f;
  float depth = depthMap[locId];
  if(fabs(depth + 1) <= EPSILON || length(normalMap[locId]) <= EPSILON)
  {
    correspondenceMap[locId] = 0;
    newPointsMask[locId] = 0;
    return;
  }

  // Otherwise, find corresponding surfels in the scene and pick the best one (if any).
  int bestSurfelIndex = -1;
  float bestSurfelConfidence = 0.0f;
  int ux = locId % depthMapWidth, uy = locId / depthMapWidth;
  for(int dy = 0; dy < supersamplingFactor; ++dy)
  {
    for(int dx = 0; dx < supersamplingFactor; ++dx)
    {
      int x = ux * supersamplingFactor + dx;
      int y = uy * supersamplingFactor + dy;
      int surfelIndex = indexMap[y * depthMapWidth * supersamplingFactor + x] - 1;
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
}

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void find_mergeable_surfel(int locId, const unsigned int *indexMap, int indexMapWidth, int indexMapHeight, const unsigned int *correspondenceMap, const TSurfel *surfels,
                                  float stableSurfelConfidence)
{
  // Look up the surfel at the current location. If there isn't one, early out.
  int surfelIndex = indexMap[locId] - 1;
  if(surfelIndex == -1) return;
  const TSurfel surfel = surfels[surfelIndex];

  // Determine whether the surfel itself can be the merge target (it needs to be stable and to have been updated this frame).
  bool surfelIsMergeTarget = surfel.confidence >= stableSurfelConfidence && correspondenceMap[locId] > 0;

  // For each neighbour of the current location that has a higher raster index:
  int x = locId % indexMapWidth, y = locId / indexMapWidth;
  int neighbourX[] = { x + 1, x, x + 1 };
  int neighbourY[] = { y, y + 1, y + 1 };
  int bestNeighbourIndex = -1;

  for(int i = 0; i < 3; ++i)
  {
    // If the neighbour is out of range, continue.
    if(neighbourX[i] >= indexMapWidth || neighbourY[i] >= indexMapHeight) continue;

    // Look up the surfel (if any) at the neighbour. If there isn't one, continue.
    int neighbourLocId = neighbourY[i] * indexMapWidth + neighbourX[i];
    int neighbourSurfelIndex = indexMap[neighbourLocId] - 1;
    if(neighbourSurfelIndex == -1) continue;
    const TSurfel neighbourSurfel = surfels[neighbourSurfelIndex];

    // If we don't have a valid merge target, continue.
    if(!surfelIsMergeTarget && !(neighbourSurfel.confidence >= stableSurfelConfidence && correspondenceMap[neighbourLocId] > 0)) continue;

    // Check the positions, normals and radii, and update the best neighbour index if the check passes.
    // TODO
  }

  // If there was a best neighbour:
  if(bestNeighbourIndex != -1)
  {
    // Set the lower-indexed entry in the merge map to the higher index.
  }
}

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void fuse_matched_point(int locId, const unsigned int *correspondenceMap, const Matrix4f& T, int timestamp,
                               const Vector4f *vertexMap, const Vector3f *normalMap, const float *radiusMap, const Vector4u *colourMap,
                               int depthMapWidth, int depthMapHeight, int colourMapWidth, int colourMapHeight,
                               const Matrix4f& depthToRGB, const Vector4f& projParamsRGB, float deltaRadius,
                               bool useGaussianSampleConfidence, float gaussianConfidenceSigma, TSurfel *surfels)
{
  const float alpha = useGaussianSampleConfidence ? calculate_gaussian_sample_confidence(locId, depthMapWidth, depthMapHeight, gaussianConfidenceSigma) : 1.0f;

  int surfelIndex = correspondenceMap[locId] - 1;
  if(surfelIndex >= 0)
  {
    TSurfel surfel = surfels[surfelIndex];
    const float newConfidence = surfel.confidence + alpha;

    const float r = radiusMap[locId];
    if(r <= (1.0f + deltaRadius) * surfel.radius)
    {
      const Vector3f v = vertexMap[locId].toVector3();
      const Vector3f newPosition = transform_point(T, v);

#if DEBUG_CORRESPONDENCES
      surfel.newPosition = newPosition;
      surfel.oldPosition = surfel.position;
#endif

      surfel.position = (surfel.confidence * surfel.position + alpha * newPosition) / newConfidence;
      surfel.normal = (surfel.confidence * surfel.normal + alpha * transform_normal(T, normalMap[locId])) / newConfidence;
      surfel.normal /= length(surfel.normal);
      surfel.radius = (surfel.confidence * surfel.radius + alpha * r) / newConfidence;

      Vector3u oldColour = SurfelColourManipulator<TSurfel::hasColourInformation>::read(surfel);
      Vector3u newColour = compute_colour(v, depthToRGB, projParamsRGB, colourMap, colourMapWidth, colourMapHeight);
      Vector3u colour = ((surfel.confidence * oldColour.toFloat() + alpha * newColour.toFloat()) / newConfidence).toUChar();
      SurfelColourManipulator<TSurfel::hasColourInformation>::write(surfel, colour);
    }

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
inline void mark_for_removal_if_unstable(int surfelId, const TSurfel *surfels, int timestamp, float stableSurfelConfidence,
                                         int unstableSurfelPeriod, unsigned int *surfelRemovalMask)
{
  TSurfel surfel = surfels[surfelId];
  if(surfel.confidence < stableSurfelConfidence && timestamp - surfel.timestamp > unstableSurfelPeriod)
  {
    surfelRemovalMask[surfelId] = 1;
  }
}

}
