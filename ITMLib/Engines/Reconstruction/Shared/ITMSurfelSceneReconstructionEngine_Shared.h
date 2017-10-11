// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#pragma once

#include "../../../Objects/Camera/ITMIntrinsics.h"
#include "../../../Objects/Scene/ITMRepresentationAccess.h"

namespace ITMLib
{

//#################### ENUMERATIONS ####################

/**
 * \brief The values of this enumeration specify the different ways of computing the radius of the combined surfel when merging two input surfels.
 */
enum RadiusCombinationMode
{
  /** Compute the radius of the combined surfel as a confidence-weighted average of the radii of the two input surfels. */
  RCM_CONFIDENCEWEIGHTEDAVERAGE,

  /** Compute the radius of the combined surfel as the distance between the positions of the two input surfels. */
  RCM_DISTANCE,
};

//#################### HELPERS ####################

_CPU_AND_GPU_CODE_ Vector3f transform_normal(const Matrix4f& T, const Vector3f& n);
_CPU_AND_GPU_CODE_ Vector3f transform_point(const Matrix4f& T, const Vector3f& p);

/**
 * \brief Calculates a Gaussian-based confidence value for a depth sample.
 *
 * See p.4 of "Real-time 3D Reconstruction in Dynamic Scenes using Point-based Fusion" (Keller et al.).
 *
 * \param locId   The raster position of the sample in the depth image.
 * \param width   The width of the depth image.
 * \param height  The height of the depth image.
 * \param sigma   The standard deviation of the Gaussian.
 * \return        The calculated confidence value.
 */
_CPU_AND_GPU_CODE_
inline float calculate_gaussian_sample_confidence(int locId, int width, int height, float sigma)
{
  // Calculate the normalised radial distance of the depth sample from the camera centre.
  const int x = locId % width, y = locId / width;
  const float halfW = width / 2.0f, halfH = height / 2.0f;
  const float dx = fabs(x - halfW), dy = fabs(y - halfH);
  const float gamma = sqrtf((dx * dx + dy * dy) / (halfW * halfW + halfH * halfH));

  // Calculate and return the confidence value itself.
  return expf(-gamma*gamma) / (2*sigma*sigma);
}

/**
 * \brief Calculates the colour to assign to a surfel.
 *
 * \param depthPos3D      The 3D position of the surfel in live 3D depth coordinates, as calculated by back-projecting from the live 2D depth image.
 * \param depthToRGB      A transformation mapping live 3D depth coordinates to live 3D colour coordinates.
 * \param projParamsRGB   The intrinsic parameters of the colour camera.
 * \param colourMap       The live 2D colour image.
 * \param colourMapWidth  The width of the colour map.
 * \param colourMapHeight The height of the colour map.
 * \return                The colour to assign to the surfel.
 */
_CPU_AND_GPU_CODE_
inline Vector3u compute_surfel_colour(const Vector3f& depthPos3D, const Matrix4f& depthToRGB, const Vector4f& projParamsRGB,
                                      const Vector4u *colourMap, int colourMapWidth, int colourMapHeight)
{
  // Transform the surfel's position into live 3D colour coordinates and project it onto the colour map.
  Vector3f rgbPos3D = transform_point(depthToRGB, depthPos3D);
  int x = static_cast<int>(projParamsRGB.x * rgbPos3D.x / rgbPos3D.z + projParamsRGB.z + 0.5f);
  int y = static_cast<int>(projParamsRGB.y * rgbPos3D.y / rgbPos3D.z + projParamsRGB.w + 0.5f);

  // If the projected position is within the bounds of the colour map, read the colour value for the surfel; if not, default to black.
  Vector3u colour((uchar)0);
  if(x >= 0 && x < colourMapWidth && y >= 0 && y < colourMapHeight)
  {
    colour = colourMap[y * colourMapWidth + x].toVector3();
  }

  return colour;
}

/**
 * \brief Makes a surfel corresponding to a pixel in the live 2D depth image.
 *
 * \param locId                       The raster position of the pixel in the live 2D depth image.
 * \param T                           A transformation from live 3D depth coordinates to global coordinates.
 * \param vertexMap                   The live point cloud, created by back-projecting the pixels in the live 2D depth image.
 * \param normalMap                   The normals computed for the points in the live point cloud.
 * \param radiusMap                   The radii computed for the points in the live point cloud.
 * \param colourMap                   The live 2D colour image.
 * \param depthMapWidth               The width of the live 2D depth image.
 * \param depthMapHeight              The height of the live 2D depth image.
 * \param colourMapWidth              The width of the live 2D colour image.
 * \param colourMapHeight             The height of the live 2D colour image.
 * \param depthToRGB                  A transformation mapping live 3D depth coordinates to live 3D RGB coordinates.
 * \param projParamsRGB               The intrinsic parameters of the colour camera.
 * \param useGaussianSampleConfidence Whether or not to use a Gaussian-weighted sample confidence as described in the Keller paper.
 * \param gaussianConfidenceSigma     The sigma value for the Gaussian used when calculating the sample confidence.
 * \param maxSurfelRadius             The maximum radius a surfel is allowed to have.
 * \param timestamp                   The current timestamp (i.e. frame number).
 * \return                            The constructed surfel.
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline TSurfel make_surfel(int locId, const Matrix4f& T, const Vector4f *vertexMap, const Vector3f *normalMap, const float *radiusMap, const Vector4u *colourMap,
                           int depthMapWidth, int depthMapHeight, int colourMapWidth, int colourMapHeight, const Matrix4f& depthToRGB,
                           const Vector4f& projParamsRGB, bool useGaussianSampleConfidence, float gaussianConfidenceSigma, float maxSurfelRadius,
                           int timestamp)
{
  TSurfel surfel;
  const Vector3f v = vertexMap[locId].toVector3();
  surfel.position = transform_point(T, v);
  surfel.normal = transform_normal(T, normalMap[locId]);
  surfel.radius = radiusMap[locId];
  if(surfel.radius > maxSurfelRadius) surfel.radius = maxSurfelRadius;
  SurfelColourManipulator<TSurfel::hasColourInformation>::write(surfel, compute_surfel_colour(v, depthToRGB, projParamsRGB, colourMap, colourMapWidth, colourMapHeight));
  surfel.confidence = useGaussianSampleConfidence ? calculate_gaussian_sample_confidence(locId, depthMapWidth, depthMapHeight, gaussianConfidenceSigma) : 1.0f;
  surfel.timestamp = timestamp;
  return surfel;
}

/**
 * \brief Calculates the result of merging one surfel into another.
 *
 * \param target                The target surfel.
 * \param source                The source surfel.
 * \param maxSurfelRadius       The maximum radius a surfel is allowed to have.
 * \param shouldMergeProperties Whether or not to merge the surfels' properties (position, normal, radius and colour) rather than just the confidence values and timestamps.
 * \param radiusCombinationMode The way in which to compute the radius of the combined surfel.
 * \return                      The surfel resulting from the merge.
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline TSurfel merge_surfels(const TSurfel& target, const TSurfel& source, float maxSurfelRadius, bool shouldMergeProperties, RadiusCombinationMode radiusCombinationMode)
{
  TSurfel result = target;
  const float confidence = result.confidence + source.confidence;

  if(shouldMergeProperties)
  {
#if DEBUG_CORRESPONDENCES
    result.newPosition = source.position;
    result.oldPosition = result.position;
#endif

    // Compute confidence-weighted averages of the positions and normals of the input surfels. Note that this can
    // result in a combined normal that is no longer unit length, so we must renormalise it accordingly.
    result.position = (result.confidence * result.position + source.confidence * source.position) / confidence;
    result.normal = (result.confidence * result.normal + source.confidence * source.normal) / confidence;
    result.normal /= length(result.normal);

    // Compute the radius of the combined surfel.
    switch(radiusCombinationMode)
    {
      case RCM_DISTANCE:
        result.radius = length(target.position - source.position);
        break;
      case RCM_CONFIDENCEWEIGHTEDAVERAGE:
      default:
        result.radius = (result.confidence * result.radius + source.confidence * source.radius) / confidence;
        break;
    }

    // Ensure that the radius of the combined surfel does not exceed the maximum permitted radius.
    if(result.radius > maxSurfelRadius) result.radius = maxSurfelRadius;

    // Compute a confidence-weighted average of the colours of the input surfels.
    Vector3u resultColour = SurfelColourManipulator<TSurfel::hasColourInformation>::read(result);
    Vector3u sourceColour = SurfelColourManipulator<TSurfel::hasColourInformation>::read(source);
    Vector3u colour = ((result.confidence * resultColour.toFloat() + source.confidence * sourceColour.toFloat()) / confidence).toUChar();
    SurfelColourManipulator<TSurfel::hasColourInformation>::write(result, colour);
  }

  result.confidence = confidence;
  if(source.timestamp > result.timestamp) result.timestamp = source.timestamp;

  return result;
}

/**
 * \brief Applies a rigid-body transformation to a normal vector.
 *
 * \param T The 4x4 matrix representing the transformation.
 * \param n The normal vector to which to apply the transformation.
 * \return  The result of applying the transformation to the normal vector.
 */
_CPU_AND_GPU_CODE_
inline Vector3f transform_normal(const Matrix4f& T, const Vector3f& n)
{
  Vector4f v(n.x, n.y, n.z, 0.0f);
  return (T * v).toVector3();
}

/**
 * \brief Applies a rigid-body transformation to a point vector.
 *
 * \param T The 4x4 matrix representing the transformation.
 * \param p The point vector to which to apply the transformation.
 * \return  The result of applying the transformation to the point vector.
 */
_CPU_AND_GPU_CODE_
inline Vector3f transform_point(const Matrix4f& T, const Vector3f& p)
{
  Vector4f v(p.x, p.y, p.z, 1.0f);
  return (T * v).toVector3();
}

/**
 * \brief Attempts to read the point in the live point cloud that was created by back-projecting from pixel (x,y) in the live 2D depth image.
 *
 * \param x         The x coordinate of the depth image pixel.
 * \param y         The y coordinate of the depth image pixel.
 * \param vertexMap The live point cloud, created by back-projecting the pixels in the live 2D depth image.
 * \param width     The width of the depth image.
 * \param height    The height of the depth image.
 * \param result    A point into which to store the result, if available.
 * \return          true, if the (x,y) coordinates are within the bounds of the depth image and there was a
 *                  valid depth value at that point, or false otherwise.
 */
_CPU_AND_GPU_CODE_
inline bool try_read_vertex(int x, int y, const Vector4f *vertexMap, int width, int height, Vector3f& result)
{
  // If the (x,y) coordinates are within the bounds of the depth image:
  if(x >= 0 && x < width && y >= 0 && y < height)
  {
    // Look up the back-projected point in the point cloud.
    Vector4f v = vertexMap[y * width + x];

    // If it's valid (if its w component is > 0), return it.
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
 * \brief Adds a new surfel (corresponding to a pixel in the live 2D depth image) to the scene.
 *
 * \param locId                       The raster position of the pixel in the live 2D depth image.
 * \param T                           A transformation from live 3D depth coordinates to global coordinates.
 * \param timestamp                   The current timestamp (i.e. frame number).
 * \param newPointsMask               A mask indicating the pixels in the live 2D depth image for which new surfels are to be added.
 * \param newPointsPrefixSum          A prefix sum computed from the new points mask (this tells us the array locations into which to write the new surfels).
 * \param vertexMap                   The live point cloud, created by back-projecting the pixels in the live 2D depth image.
 * \param normalMap                   The normals computed for the points in the live point cloud.
 * \param radiusMap                   The radii computed for the points in the live point cloud.
 * \param colourMap                   The live 2D colour image.
 * \param depthMapWidth               The width of the live 2D depth image.
 * \param depthMapHeight              The height of the live 2D depth image.
 * \param colourMapWidth              The width of the live 2D colour image.
 * \param colourMapHeight             The height of the live 2D colour image.
 * \param depthToRGB                  A transformation mapping live 3D depth coordinates to live 3D RGB coordinates.
 * \param projParamsRGB               The intrinsic parameters of the colour camera.
 * \param useGaussianSampleConfidence Whether or not to use a Gaussian-weighted sample confidence as described in the Keller paper.
 * \param gaussianConfidenceSigma     The sigma value for the Gaussian used when calculating the sample confidence.
 * \param maxSurfelRadius             The maximum radius a surfel is allowed to have.
 * \param newSurfels                  The start of the chunk of memory allocated for the new surfels in the surfel scene.
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void add_new_surfel(int locId, const Matrix4f& T, int timestamp, const unsigned short *newPointsMask, const unsigned int *newPointsPrefixSum,
                           const Vector4f *vertexMap, const Vector3f *normalMap, const float *radiusMap, const Vector4u *colourMap,
                           int depthMapWidth, int depthMapHeight, int colourMapWidth, int colourMapHeight, const Matrix4f& depthToRGB,
                           const Vector4f& projParamsRGB, bool useGaussianSampleConfidence, float gaussianConfidenceSigma, float maxSurfelRadius,
                           TSurfel *newSurfels)
{
  // If a new surfel is to be added for this pixel in the live 2D depth image:
  if(newPointsMask[locId])
  {
    // Make the surfel.
    TSurfel surfel = make_surfel<TSurfel>(
      locId, T, vertexMap, normalMap, radiusMap, colourMap, depthMapWidth, depthMapHeight, colourMapWidth, colourMapHeight,
      depthToRGB, projParamsRGB, useGaussianSampleConfidence, gaussianConfidenceSigma, maxSurfelRadius, timestamp
    );

    // Write it into the correct position in the chunk of memory allocated for new surfels.
    newSurfels[newPointsPrefixSum[locId]] = surfel;
  }
}

/**
 * \brief Calculates an estimate of the surface normal for the point at the specified raster position in the vertex map.
 *
 * \param locId       The raster position in the vertex map of the point for which to estimate the surface normal.
 * \param vertexMap   The live point cloud, created by back-projecting the pixels in the live 2D depth image.
 * \param width       The width of the vertex map.
 * \param height      The height of the vertex map.
 * \param normalMap   A map in which to store the estimated surface normals.
 */
_CPU_AND_GPU_CODE_
inline void calculate_normal(int locId, const Vector4f *vertexMap, int width, int height, Vector3f *normalMap)
{
  // Note: This approach works for the moment, but there may be ways of improving it.
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
 * \brief Calculates the radius to give a point in the live point cloud corresponding to a pixel in the live 2D depth image.
 *
 * \param locId       The raster position of the pixel in the live 2D depth image.
 * \param depthMap    The live 2D depth image.
 * \param normalMap   The normals computed for the points in the live point cloud.
 * \param intrinsics  The intrinsic parameters of the depth camera.
 * \param radiusMap   A map into which to write the radii computed for the points in the live point cloud.
 */
_CPU_AND_GPU_CODE_
inline void calculate_radius(int locId, const float *depthMap, const Vector3f *normalMap, const ITMIntrinsics& intrinsics, float *radiusMap)
{
  float r = 0.0f;
  Vector3f n = normalMap[locId];

  if(length(n) > 0.0f)
  {
    // The intuition behind the radius calculation is that you want the surfel to fully cover a single pixel on the image plane (at distance f).
    // To do this, it needs to have a projected radius of sqrt(2). Projecting it down onto the image plane from distance d means multiplying its
    // radius by f / d, hence its unprojected radius should be sqrt(2) / (f / d) = sqrt(2) * d / f. Note that this is the radius calculation used
    // in the "Dense Planar SLAM" paper, rather than the one used in the original Keller paper.
    float d = depthMap[locId];
    float f = 0.5f * (intrinsics.projectionParamsSimple.fx + intrinsics.projectionParamsSimple.fy);
    r = sqrt(2.0f) * d / f;
  }

  radiusMap[locId] = r;
}

/**
 * \brief Back-projects a point in the live 2D depth image to find its position in live 3D depth coordinates.
 *
 * \param locId       The raster position in the live 2D depth image of the point to back-project.
 * \param width       The width of the depth image.
 * \param intrinsics  The intrinsic parameters of the depth camera.
 * \param depthMap    The live 2D depth image.
 * \param vertexMap   A map in which to store the back-projections of the pixels in the live 2D depth image.
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
  if(fabs(depth + 1) > EPSILON) // i.e. if(depth != -1)
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
 * \brief Clears the surfel merge indicated by the specified entry in the merge target map.
 *
 * \param locId           The raster position of the entry in the merge target map.
 * \param mergeTargetMap  The merge target map.
 */
_CPU_AND_GPU_CODE_
inline void clear_merge_target(int locId, unsigned int *mergeTargetMap)
{
  mergeTargetMap[locId] = 0;
}

/**
 * \brief Clears the entry of the specified surfel in the surfel removal mask.
 *
 * \param surfelId            The ID of the surfel whose entry in the surfel removal mask is to be cleared.
 * \param surfelRemovalMask   A mask used to indicate which surfels should be removed in the next removal pass.
 */
_CPU_AND_GPU_CODE_
inline void clear_removal_mask_entry(int surfelId, unsigned int *surfelRemovalMask)
{
  surfelRemovalMask[surfelId] = 0;
}

/**
 * \brief Attempts to find a surfel in the scene into which the point denoted by the specified raster position in the live point cloud can be fused.
 *
 * If no corresponding surfel can be found for a point, it will be marked as a new point in the new points mask, and a new surfel will be created for it.
 *
 * \param locId                 The raster position in the live point cloud for whose point we want to try to find a corresponding surfel in the scene.
 * \param invT                  A transformation mapping global coordinates to live 3D depth coordinates.
 * \param depthMap              The live 2D depth image.
 * \param depthMapWidth         The width of the live 2D depth image.
 * \param normalMap             The normals computed for the points in the live point cloud.
 * \param indexImage            The surfel index image.
 * \param supersamplingFactor   The factor by which to supersample (in each axis) the index image used for finding surfel correspondences.
 * \param surfels               The surfels in the scene.
 * \param correspondenceMap     The correspondence map, each pixel of which indicates the surfel (if any) with which the relevant point in the live point cloud has been matched.
 * \param newPointsMask         A mask indicating the pixels in the live 2D depth image for which new surfels are to be added.
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void find_corresponding_surfel(int locId, const Matrix4f& invT, const float *depthMap, int depthMapWidth, const Vector3f *normalMap, const unsigned int *indexImage,
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
      int surfelIndex = indexImage[y * depthMapWidth * supersamplingFactor + x] - 1;
      if(surfelIndex >= 0)
      {
        // TODO: Make this slightly more sophisticated, as per the paper.
        TSurfel surfel = surfels[surfelIndex];
        Vector3f liveSurfelPos = transform_point(invT, surfel.position);
        float surfelDepth = liveSurfelPos.z;

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
 * \brief Attempts to find a surfel in the scene that can be merged into the surfel (if any) denoted by the specified raster position in the surfel index image.
 *
 * \param locId                   The raster position in the surfel index image for whose surfel (if any) we want to find a merge source.
 * \param indexImage              The index image.
 * \param indexImageWidth         The width of the index image.
 * \param indexImageHeight        The height of the index image.
 * \param correspondenceMap       The correspondence map, each pixel of which indicates the surfel (if any) with which the relevant point in the live point cloud has been matched.
 * \param surfels                 The surfels in the scene.
 * \param stableSurfelConfidence  The confidence value a surfel must have in order for it to be considered "stable".
 * \param maxMergeDist            The maximum distance allowed between a pair of surfels if they are to be merged.
 * \param maxMergeAngle           The maximum angle allowed between the normals of a pair of surfels if they are to be merged.
 * \param minRadiusOverlapFactor  The minimum factor by which the radii of a pair of surfels must overlap if they are to be merged.
 * \param mergeTargetMap          The merge target map.
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void find_mergeable_surfel(int locId, const unsigned int *indexImage, int indexImageWidth, int indexImageHeight, const unsigned int *correspondenceMap, const TSurfel *surfels,
                                  float stableSurfelConfidence, float maxMergeDist, float maxMergeAngle, float minRadiusOverlapFactor, unsigned int *mergeTargetMap)
{
  // Look up the surfel at the current location. If there isn't one, early out.
  int surfelIndex = indexImage[locId] - 1;
  if(surfelIndex == -1) return;
  const TSurfel surfel = surfels[surfelIndex];

  // Determine whether the surfel itself can justify the merge (it needs to be stable and to have been updated this frame).
  bool surfelCanJustify = surfel.confidence >= stableSurfelConfidence && correspondenceMap[locId] > 0;

  // For each neighbour of the current location that has a higher raster index:
  int x = locId % indexImageWidth, y = locId / indexImageWidth;
  int neighbourX[] = { x + 1, x - 1, x, x + 1 };
  int neighbourY[] = { y, y + 1, y + 1, y + 1 };
  int bestMergeSource = -1;

  for(int i = 0; i < 4; ++i)
  {
    // If the neighbour is out of range, continue.
    if(neighbourX[i] >= indexImageWidth || neighbourY[i] >= indexImageHeight) continue;

    // Look up the surfel (if any) at the neighbour. If there isn't one, or it's the same as this surfel, continue.
    int neighbourLocId = neighbourY[i] * indexImageWidth + neighbourX[i];
    int neighbourSurfelIndex = indexImage[neighbourLocId] - 1;
    if(neighbourSurfelIndex == -1 || neighbourSurfelIndex == surfelIndex) continue;
    const TSurfel neighbourSurfel = surfels[neighbourSurfelIndex];

    // If the merge cannot be justified, continue.
    if(!surfelCanJustify && !(neighbourSurfel.confidence >= stableSurfelConfidence && correspondenceMap[neighbourLocId] > 0)) continue;

    // If the difference in positions and the angle between the normals are sufficiently small, and the radii significantly overlap, update the best merge source.
    float dist = length(surfel.position - neighbourSurfel.position);
    float angle = acosf(dot(surfel.normal, neighbourSurfel.normal));
    if(dist <= maxMergeDist && angle <= maxMergeAngle && dist * minRadiusOverlapFactor <= surfel.radius + neighbourSurfel.radius)
    {
      bestMergeSource = neighbourLocId;
    }
  }

  // If there was a best merge source, record the current location as its merge target.
  if(bestMergeSource != -1) mergeTargetMap[bestMergeSource] = locId + 1;
}

/**
 * \brief Fuses a point in the live point cloud into the surfel in the scene with which it has been matched.
 *
 * \param locId                       The raster position of the point in the correspondence map.
 * \param correspondenceMap           The correspondence map, each pixel of which indicates the surfel (if any) with which the relevant point in the live point cloud has been matched.
 * \param T                           A transformation from live 3D depth coordinates to global coordinates.
 * \param timestamp                   The current timestamp (i.e. frame number).
 * \param vertexMap                   The live point cloud, created by back-projecting the pixels in the live 2D depth image.
 * \param normalMap                   The normals computed for the points in the live point cloud.
 * \param radiusMap                   The radii computed for the points in the live point cloud.
 * \param colourMap                   The live 2D colour image.
 * \param depthMapWidth               The width of the depth map.
 * \param depthMapHeight              The height of the depth map.
 * \param colourMapWidth              The width of the colour map.
 * \param colourMapHeight             The height of the colour map.
 * \param depthToRGB                  A transformation mapping live 3D depth coordinates to live 3D colour coordinates.
 * \param projParamsRGB               The intrinsic parameters of the colour camera.
 * \param deltaRadius                 The maximum fraction by which a new surfel can have a larger radius than the surfel into which it is being fused if full fusion is to occur.
 * \param useGaussianSampleConfidence Whether or not to use a Gaussian-weighted sample confidence as described in the Keller paper.
 * \param gaussianConfidenceSigma     The sigma value for the Gaussian used when calculating the sample confidence.
 * \param maxSurfelRadius             The maximum radius a surfel is allowed to have.
 * \param surfels                     The surfels in the scene.
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void fuse_matched_point(int locId, const unsigned int *correspondenceMap, const Matrix4f& T, int timestamp,
                               const Vector4f *vertexMap, const Vector3f *normalMap, const float *radiusMap, const Vector4u *colourMap,
                               int depthMapWidth, int depthMapHeight, int colourMapWidth, int colourMapHeight,
                               const Matrix4f& depthToRGB, const Vector4f& projParamsRGB, float deltaRadius,
                               bool useGaussianSampleConfidence, float gaussianConfidenceSigma, float maxSurfelRadius,
                               TSurfel *surfels)
{
  int surfelIndex = correspondenceMap[locId] - 1;
  if(surfelIndex >= 0)
  {
    TSurfel surfel = surfels[surfelIndex];
    TSurfel newSurfel = make_surfel<TSurfel>(
      locId, T, vertexMap, normalMap, radiusMap, colourMap, depthMapWidth, depthMapHeight, colourMapWidth, colourMapHeight,
      depthToRGB, projParamsRGB, useGaussianSampleConfidence, gaussianConfidenceSigma, maxSurfelRadius, timestamp
    );

    bool shouldMergeProperties = newSurfel.radius <= (1.0f + deltaRadius) * surfel.radius;
    surfel = merge_surfels(surfel, newSurfel, maxSurfelRadius, shouldMergeProperties, RCM_CONFIDENCEWEIGHTEDAVERAGE);

    surfels[surfelIndex] = surfel;
  }
}

/**
 * \brief Marks a surfel for removal if it has been unstable for longer than a specified period of time.
 *
 * \param surfelId                The ID of the surfel.
 * \param surfels                 The surfels in the scene.
 * \param timestamp               The current timestamp (i.e. frame number).
 * \param stableSurfelConfidence  The confidence value a surfel must have in order for it to be considered "stable".
 * \param unstableSurfelPeriod    The number of time steps a surfel is allowed to be unstable without being updated before being removed.
 * \param surfelRemovalMask       A mask used to indicate which surfels should be removed in the next removal pass.
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

/**
 * \brief Performs the surfel merge (if any) indicated by an entry in the merge target map.
 *
 * \param locId             The raster position of the entry in the merge target map.
 * \param mergeTargetMap    The merge target map.
 * \param surfels           The surfels in the scene.
 * \param surfelRemovalMask A mask used to indicate which surfels should be removed in the next removal pass.
 * \param indexImage        The surfel index image.
 * \param maxSurfelRadius   The maximum radius a surfel is allowed to have.
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void perform_surfel_merge(int locId, unsigned int *mergeTargetMap, TSurfel *surfels, unsigned int *surfelRemovalMask, const unsigned int *indexImage, float maxSurfelRadius)
{
  // If there's no target for the merge, early out.
  int mergeTarget = mergeTargetMap[locId] - 1;
  if(mergeTarget == -1) return;

  // Look up the source and target surfels. They should always exist, but if they don't then ignore it and early out.
  int sourceSurfelIndex = indexImage[locId] - 1, targetSurfelIndex = indexImage[mergeTarget] - 1;
  if(sourceSurfelIndex == -1 || targetSurfelIndex == -1) return;

  // Merge the source surfel into the target surfel.
  bool shouldMergeProperties = true;
  TSurfel source = surfels[sourceSurfelIndex], target = surfels[targetSurfelIndex];
  TSurfel merged = merge_surfels(target, source, maxSurfelRadius, shouldMergeProperties, RCM_CONFIDENCEWEIGHTEDAVERAGE);
  surfels[targetSurfelIndex] = merged;

  // Mark the source surfel for removal.
  surfelRemovalMask[sourceSurfelIndex] = 1;
}

/**
 * \brief Prevents the target of any merge at the specified location in the merge target map from being the source of a separate merge.
 *
 * In other words, if surfel a is due to be merged into surfel b, prevent b from merging into any other surfel c.
 *
 * \param locId           The location in the merge target map for which to prevent a merge chain.
 * \param mergeTargetMap  The merge target map.
 */
_CPU_AND_GPU_CODE_
inline void prevent_merge_chain(int locId, unsigned int *mergeTargetMap)
{
  int mergeTarget = mergeTargetMap[locId] - 1;
  if(mergeTarget >= 0)
  {
    mergeTargetMap[mergeTarget] = 0;
  }
}

}
