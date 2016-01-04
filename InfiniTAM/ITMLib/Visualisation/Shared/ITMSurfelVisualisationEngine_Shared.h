// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Objects/Scene/ITMRepresentationAccess.h"

namespace ITMLib
{

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
inline void copy_surfel_to_buffers(int surfelId, const TSurfel *surfels, float *positions, unsigned char *normals, unsigned char *colours)
{
  TSurfel surfel = surfels[surfelId];
  int offset = surfelId * 3;

  Vector3f p = surfel.position;
  positions[offset] = p.x;
  positions[offset+1] = p.y;
  positions[offset+2] = p.z;

  // FIXME: Borrowed from drawPixelNormal - refactor.
  Vector3f n = surfel.normal;
  normals[offset] = (uchar)((0.3f + (-n.x + 1.0f)*0.35f)*255.0f);
  normals[offset+1] = (uchar)((0.3f + (-n.y + 1.0f)*0.35f)*255.0f);
  normals[offset+2] = (uchar)((0.3f + (-n.z + 1.0f)*0.35f)*255.0f);

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
inline void project_to_surfel_index_image(int surfelId, const TSurfel *surfels, const Matrix4f& invT, const ITMIntrinsics& intrinsics, int indexImageWidth, int indexImageHeight,
                                          int scaleFactor, unsigned int *surfelIndexImage, int *depthBuffer)
{
  // Convert the surfel point into the coordinates of the current frame using v_i = T_i^{-1} v_i^g.
  Vector3f p = surfels[surfelId].position;
  Vector4f vg(p.x, p.y, p.z, 1.0f);
  Vector4f v = invT * vg;

  // Project the point onto the image plane of the current frame.
  float ux = intrinsics.projectionParamsSimple.fx * v.x / v.z + intrinsics.projectionParamsSimple.px;
  float uy = intrinsics.projectionParamsSimple.fy * v.y / v.z + intrinsics.projectionParamsSimple.py;

  // Convert the projected point into index map coordinates.
  int x = static_cast<int>(ux * scaleFactor + 0.5f);
  int y = static_cast<int>(uy * scaleFactor + 0.5f);

  if(0 <= x && x < indexImageWidth && 0 <= y && y < indexImageHeight)
  {
    // Write the surfel ID + 1 into the surfel index image.
    surfelIndexImage[y * indexImageWidth + x] = static_cast<unsigned int>(surfelId + 1);
  }
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
                       Vector4u *outputImage)
{
  unsigned char c = 0;

  int surfelIndex = surfelIndexImage[locId] - 1;
  if(surfelIndex >= 0)
  {
    Vector3f p = surfels[surfelIndex].position;
    float dx = abs(cameraPosition.x - p.x);
    float dy = abs(cameraPosition.y - p.y);
    float dz = abs(cameraPosition.z - p.z);
    float value = sqrt(dx * dx + dy * dy + dz * dz);
    if(value >= 0)
    {
      // FIXME: This should be tidied up in due course.
      int i = static_cast<int>(255 * value);
      if(i > 255) i = 255;
      c = static_cast<unsigned char>(i);
    }
  }

  outputImage[locId] = Vector4u(c, c, c, 255);
}

}
