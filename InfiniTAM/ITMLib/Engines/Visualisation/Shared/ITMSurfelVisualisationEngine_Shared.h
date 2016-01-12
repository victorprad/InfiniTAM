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

#if DEBUG_CORRESPONDENCES
/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void copy_correspondences_to_buffer(int surfelId, const TSurfel *surfels, float *correspondences)
{
  TSurfel surfel = surfels[surfelId];
  int offset = surfelId * 6;

  Vector3f p = surfel.position;
  Vector3f cp = surfel.correspondingSurfelPosition;

  correspondences[offset] = p.x;
  correspondences[offset+1] = p.y;
  correspondences[offset+2] = p.z;
  correspondences[offset+3] = cp.x;
  correspondences[offset+4] = cp.y;
  correspondences[offset+5] = cp.z;
}
#endif

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void copy_surfel_data_to_icp_maps(int locId, const TSurfel *surfels, const unsigned int *surfelIndexImage, Vector4f *pointsMap, Vector4f *normalsMap)
{
  int surfelIndex = surfelIndexImage[locId] - 1;
  if(surfelIndex >= 0)
  {
    TSurfel surfel = surfels[surfelIndex];
    const Vector3f& p = surfel.position;
    const Vector3f& n = surfel.normal;
    pointsMap[locId] = Vector4f(p.x, p.y, p.z, 1.0f);
    normalsMap[locId] = Vector4f(n.x, n.y, n.z, 0.0f);
  }
  else
  {
    Vector4f dummy;
    dummy.x = dummy.y = dummy.z = 0.0f; dummy.w = -1.0f;
    pointsMap[locId] = dummy;
    normalsMap[locId] = dummy;
  }
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
                                          int& locId, int& z)
{
  // Convert the surfel point into the coordinates of the current frame using v_i = T_i^{-1} v_i^g.
  Vector3f p = surfel.position;
  Vector4f vg(p.x, p.y, p.z, 1.0f);
  Vector4f v = invT * vg;

  // If the point isn't in front of the viewer, early out.
  if(v.z <= 0) return false;

  // Project the point onto the image plane of the current frame.
  float ux = intrinsics.projectionParamsSimple.fx * v.x / v.z + intrinsics.projectionParamsSimple.px;
  float uy = intrinsics.projectionParamsSimple.fy * v.y / v.z + intrinsics.projectionParamsSimple.py;

  // Convert the projected point into index map coordinates.
  int x = static_cast<int>(ux * scaleFactor + 0.5f);
  int y = static_cast<int>(uy * scaleFactor + 0.5f);

  // If the resulting point is outside the index map, early out.
  if(x < 0 || x >= indexImageWidth || y < 0 || y >= indexImageHeight) return false;

  // Calculate the raster position of the point in the index map.
  locId = y * indexImageWidth + x;

  // Calculate an integer depth value for the point for storage in the depth buffer.
  z = static_cast<int>(v.z * 100000);

  return true;
}

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
void shade_pixel_colour(int locId, const unsigned int *surfelIndexImage, const TSurfel *surfels, Vector4u *outputImage)
{
  Vector4u col4(0, 0, 0, 255);

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
                                           int indexImageWidth, int indexImageHeight, int scaleFactor, int *depthBuffer)
{
  int locId, z;
  if(project_surfel_to_index_image(surfels[surfelId], invT, intrinsics, indexImageWidth, indexImageHeight, scaleFactor, locId, z))
  {
#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
    atomicMin(&depthBuffer[locId], z);
#else
    // Note: No synchronisation is needed for the CPU version because it's not parallelised.
    if(z < depthBuffer[locId]) depthBuffer[locId] = z;
#endif
  }
}

/**
 * \brief TODO
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void update_index_image_for_surfel(int surfelId, const TSurfel *surfels, const Matrix4f& invT, const ITMIntrinsics& intrinsics,
                                          int indexImageWidth, int indexImageHeight, int scaleFactor, const int *depthBuffer,
                                          unsigned int *surfelIndexImage)
{
  int locId, z;
  if(project_surfel_to_index_image(surfels[surfelId], invT, intrinsics, indexImageWidth, indexImageHeight, scaleFactor, locId, z))
  {
    if(depthBuffer[locId] == z)
    {
      // Write the surfel ID + 1 into the surfel index image.
      surfelIndexImage[locId] = static_cast<unsigned int>(surfelId + 1);
    }
  }
}

}
