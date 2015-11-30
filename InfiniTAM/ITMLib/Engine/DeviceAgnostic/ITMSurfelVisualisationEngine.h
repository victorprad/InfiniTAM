// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../ORUtils/PlatformIndependence.h"

namespace ITMLib
{

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
inline void project_to_surfel_index_image(int surfelId, const TSurfel *surfels, const Matrix4f& invT, const ITMIntrinsics& intrinsics, int width, int height,
                                          unsigned long *surfelIndexImage, float *depthBuffer)
{
  // TODO
}

}
