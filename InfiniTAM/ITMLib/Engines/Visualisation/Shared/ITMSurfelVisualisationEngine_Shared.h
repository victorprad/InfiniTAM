// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/Scene/ITMRepresentationAccess.h"

namespace ITMLib
{

//#################### HELPERS ####################

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void calculate_projected_surfel_bounds(int locId, int indexImageWidth, int indexImageHeight, float radius, float z,
                                              int& cx, int& cy, int& projectedRadiusSquared, int& minX, int& minY, int& maxX, int& maxY)
{
  cx = locId % indexImageWidth, cy = locId / indexImageWidth;
  int projectedRadius = static_cast<int>(radius / z + 0.5f);
  projectedRadiusSquared = projectedRadius * projectedRadius;
  minX = cx - projectedRadius, maxX = cx + projectedRadius;
  minY = cy - projectedRadius, maxY = cy + projectedRadius;
  if(minX < 0) minX = 0;
  if(maxX >= indexImageWidth) maxX = indexImageWidth - 1;
  if(minY < 0) minY = 0;
  if(maxY >= indexImageHeight) maxY = indexImageHeight - 1;
}

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline Vector4u colourise_normal(const Vector3f& n)
{
  // FIXME: Borrowed from drawPixelNormal - refactor.
  return Vector4u(
    (uchar)((0.3f + (-n.x + 1.0f)*0.35f)*255.0f),
    (uchar)((0.3f + (-n.y + 1.0f)*0.35f)*255.0f),
    (uchar)((0.3f + (-n.z + 1.0f)*0.35f)*255.0f),
    255
  );
}

//#################### MAIN FUNCTIONS ####################

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void clear_surfel_index_image(int locId, unsigned int *surfelIndexImage, int *depthBuffer)
{
  surfelIndexImage[locId] = 0;
  depthBuffer[locId] = INT_MAX;
}

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void copy_correspondences_to_buffers(int surfelId, const TSurfel *surfels, float *newPositions, float *oldPositions, float *correspondences)
{
#if DEBUG_CORRESPONDENCES
  TSurfel surfel = surfels[surfelId];
  Vector3f np = surfel.newPosition;
  Vector3f op = surfel.oldPosition;

  int offset = surfelId * 3;
  newPositions[offset] = np.x;
  newPositions[offset+1] = np.y;
  newPositions[offset+2] = np.z;
  oldPositions[offset] = op.x;
  oldPositions[offset+1] = op.y;
  oldPositions[offset+2] = op.z;

  offset = surfelId * 6;
  correspondences[offset] = np.x;
  correspondences[offset+1] = np.y;
  correspondences[offset+2] = np.z;
  correspondences[offset+3] = op.x;
  correspondences[offset+4] = op.y;
  correspondences[offset+5] = op.z;
#endif
}

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void copy_surfel_data_to_icp_maps(int locId, const TSurfel *surfels, const unsigned int *surfelIndexImage, const Matrix4f& invT,
                                         float trackingSurfelMaxDepth, float trackingSurfelMinConfidence, Vector4f *pointsMap, Vector4f *normalsMap)
{
  int surfelIndex = surfelIndexImage[locId] - 1;
  if(surfelIndex >= 0)
  {
    TSurfel surfel = surfels[surfelIndex];
    const Vector3f& p = surfel.position;
    const Vector3f& n = surfel.normal;
    Vector3f v = transform_point(invT, p);
    if(v.z <= trackingSurfelMaxDepth || surfel.confidence >= trackingSurfelMinConfidence)
    {
      pointsMap[locId] = Vector4f(p.x, p.y, p.z, 1.0f);
      normalsMap[locId] = Vector4f(n.x, n.y, n.z, 0.0f);
      return;
    }
  }

  Vector4f dummy;
  dummy.x = dummy.y = dummy.z = 0.0f; dummy.w = -1.0f;
  pointsMap[locId] = dummy;
  normalsMap[locId] = dummy;
}

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void copy_surfel_to_buffers(int surfelId, const TSurfel *surfels, float *positions, unsigned char *normals, unsigned char *colours)
{
  TSurfel surfel = surfels[surfelId];
  int offset = surfelId * 3;

  Vector3f p = surfel.position;
  positions[offset] = p.x;
  positions[offset+1] = p.y;
  positions[offset+2] = p.z;

  Vector4u n = colourise_normal(surfel.normal);
  normals[offset] = n.x;
  normals[offset+1] = n.y;
  normals[offset+2] = n.z;

  if(colours != NULL)
  {
    Vector3u c = SurfelColourManipulator<TSurfel::hasColourInformation>::read(surfel);
    colours[offset] = c.r;
    colours[offset+1] = c.g;
    colours[offset+2] = c.b;
  }
}

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline bool project_surfel_to_index_image(const TSurfel& surfel, const Matrix4f& invT, const ITMIntrinsics& intrinsics,
                                          int indexImageWidth, int indexImageHeight, int scaleFactor,
                                          int& locId, float& z, int& scaledZ)
{
  // Convert the surfel point into the coordinates of the current frame using v_i = T_i^{-1} v_i^g.
  Vector3f p = surfel.position;
  Vector4f vg(p.x, p.y, p.z, 1.0f);
  Vector4f v = invT * vg;

  // If the point isn't in front of the viewer, early out.
  z = v.z;
  if(z <= 0.0f) return false;

  // Project the point onto the image plane of the current frame.
  float ux = intrinsics.projectionParamsSimple.fx * v.x / z + intrinsics.projectionParamsSimple.px;
  float uy = intrinsics.projectionParamsSimple.fy * v.y / z + intrinsics.projectionParamsSimple.py;

  // Convert the projected point into index map coordinates.
  int x = static_cast<int>(ux * scaleFactor + 0.5f);
  int y = static_cast<int>(uy * scaleFactor + 0.5f);

  // If the resulting point is outside the index map, early out.
  if(x < 0 || x >= indexImageWidth || y < 0 || y >= indexImageHeight) return false;

  // Calculate the raster position of the point in the index map.
  locId = y * indexImageWidth + x;

  // Calculate an integer depth value for the point for storage in the depth buffer.
  scaledZ = static_cast<int>(z * 100000);

  return true;
}

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
void shade_pixel_colour(int locId, const unsigned int *surfelIndexImage, const TSurfel *surfels, Vector4u *outputImage)
{
  Vector4u col4(0, 255, 255, 255);

  int surfelIndex = surfelIndexImage[locId] - 1;
  if(surfelIndex >= 0)
  {
    Vector3u col3 = SurfelColourManipulator<TSurfel::hasColourInformation>::read(surfels[surfelIndex]);
    col4 = Vector4u(col3.x, col3.y, col3.z, 255);
  }

  outputImage[locId] = col4;
}

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
void shade_pixel_confidence(int locId, const unsigned int *surfelIndexImage, const TSurfel *surfels, float stableSurfelConfidence, Vector4u *outputImage)
{
  Vector4u col4(0, 0, 0, 255);

  int surfelIndex = surfelIndexImage[locId] - 1;
  if(surfelIndex >= 0)
  {
    float confidence = surfels[surfelIndex].confidence;
    if(confidence > stableSurfelConfidence) confidence = stableSurfelConfidence;
    uchar g = (uchar)(255.0f * confidence / stableSurfelConfidence);
    col4 = Vector4u(255 - g, g, 0, 255);
  }

  outputImage[locId] = col4;
}

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
void shade_pixel_depth(int locId, const unsigned int *surfelIndexImage, const TSurfel *surfels, const Vector3f& cameraPosition,
                       float *outputImage)
{
  // FIXME: This should be set to a less arbitrary value.
  float value = -1.0f;

  int surfelIndex = surfelIndexImage[locId] - 1;
  if(surfelIndex >= 0)
  {
    Vector3f p = surfels[surfelIndex].position;
    float dx = abs(cameraPosition.x - p.x);
    float dy = abs(cameraPosition.y - p.y);
    float dz = abs(cameraPosition.z - p.z);
    value = sqrt(dx * dx + dy * dy + dz * dz);
  }

  outputImage[locId] = value;
}

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
void shade_pixel_grey(int locId, const unsigned int *surfelIndexImage, const TSurfel *surfels, const Vector3f& lightSource,
                      Vector4u *outputImage)
{
  const float ambient = 0.2f;
  const float lambertianCoefficient = 0.8f;

  Vector4u value((uchar)0);

  int surfelIndex = surfelIndexImage[locId] - 1;
  if(surfelIndex >= 0)
  {
    TSurfel surfel = surfels[surfelIndex];
    float NdotL = ORUtils::dot(surfel.normal, lightSource);
    float lambertian = CLAMP(NdotL, 0.0f, 1.0f);
    float intensity = ambient + lambertianCoefficient * lambertian;
    value = Vector4u((uchar)(intensity * 255.0f));
  }

  outputImage[locId] = value;
}

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
void shade_pixel_normal(int locId, const unsigned int *surfelIndexImage, const TSurfel *surfels, Vector4u *outputImage)
{
  Vector4u value((uchar)0);

  int surfelIndex = surfelIndexImage[locId] - 1;
  if(surfelIndex >= 0)
  {
    value = colourise_normal(surfels[surfelIndex].normal);
  }

  outputImage[locId] = value;
}

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void update_depth_buffer_for_surfel(int surfelId, const TSurfel *surfels, const Matrix4f& invT, const ITMIntrinsics& intrinsics,
                                           int indexImageWidth, int indexImageHeight, int scaleFactor, bool useRadii, int *depthBuffer)
{
  const TSurfel surfel = surfels[surfelId];
  int locId, scaledZ;
  float z;
  if(project_surfel_to_index_image(surfel, invT, intrinsics, indexImageWidth, indexImageHeight, scaleFactor, locId, z, scaledZ))
  {
    if(useRadii)
    {
      int cx, cy, minX, minY, maxX, maxY, projectedRadiusSquared;
      calculate_projected_surfel_bounds(
        locId, indexImageWidth, indexImageHeight, surfel.radius, z,
        cx, cy, projectedRadiusSquared, minX, minY, maxX, maxY
      );

      for(int y = minY; y <= maxY; ++y)
      {
        int yOffset = y - cy;
        int yOffsetSquared = yOffset * yOffset;

        for(int x = minX; x <= maxX; ++x)
        {
          int xOffset = x - cx;
          int xOffsetSquared = xOffset * xOffset;
          if(xOffsetSquared + yOffsetSquared > projectedRadiusSquared) continue;

          int offset = y * indexImageWidth + x;

#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
          atomicMin(&depthBuffer[offset], scaledZ);
#else
          // Note: No synchronisation is needed for the CPU version because it's not parallelised.
          if(scaledZ < depthBuffer[offset]) depthBuffer[offset] = scaledZ;
#endif
        }
      }
    }
    else
    {
#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
      atomicMin(&depthBuffer[locId], scaledZ);
#else
      // Note: No synchronisation is needed for the CPU version because it's not parallelised.
      if(scaledZ < depthBuffer[locId]) depthBuffer[locId] = scaledZ;
#endif
    }
  }
}

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void update_index_image_for_surfel(int surfelId, const TSurfel *surfels, const Matrix4f& invT, const ITMIntrinsics& intrinsics,
                                          int indexImageWidth, int indexImageHeight, int scaleFactor, const int *depthBuffer, bool useRadii,
                                          unsigned int *surfelIndexImage)
{
  const TSurfel surfel = surfels[surfelId];
  int locId, scaledZ;
  float z;
  if(project_surfel_to_index_image(surfel, invT, intrinsics, indexImageWidth, indexImageHeight, scaleFactor, locId, z, scaledZ))
  {
    unsigned int surfelIdPlusOne = static_cast<unsigned int>(surfelId + 1);

    if(useRadii)
    {
      int cx, cy, minX, minY, maxX, maxY, projectedRadiusSquared;
      calculate_projected_surfel_bounds(
        locId, indexImageWidth, indexImageHeight, surfel.radius, z,
        cx, cy, projectedRadiusSquared, minX, minY, maxX, maxY
      );

      for(int y = minY; y <= maxY; ++y)
      {
        int yOffset = y - cy;
        int yOffsetSquared = yOffset * yOffset;

        for(int x = minX; x <= maxX; ++x)
        {
          int xOffset = x - cx;
          int xOffsetSquared = xOffset * xOffset;
          if(xOffsetSquared + yOffsetSquared > projectedRadiusSquared) continue;

          int offset = y * indexImageWidth + x;

          if(depthBuffer[offset] == scaledZ)
          {
#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
            atomicMax(&surfelIndexImage[offset], surfelIdPlusOne);
#else
            // Note: No synchronisation is needed for the CPU version because it's not parallelised.
            if(surfelIdPlusOne > surfelIndexImage[offset]) surfelIndexImage[offset] = surfelIdPlusOne;
#endif
          }
        }
      }
    }
    else
    {
      if(depthBuffer[locId] == scaledZ)
      {
#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
        atomicMax(&surfelIndexImage[locId], surfelIdPlusOne);
#else
        // Note: No synchronisation is needed for the CPU version because it's not parallelised.
        if(surfelIdPlusOne > surfelIndexImage[locId]) surfelIndexImage[locId] = surfelIdPlusOne;
#endif
      }
    }
  }
}

}
