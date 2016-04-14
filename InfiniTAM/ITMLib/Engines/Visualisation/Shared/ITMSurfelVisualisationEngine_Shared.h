// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#pragma once

#include <climits>

#include "ITMSurfelVisualisationEngine_Settings.h"
#include "../../../Objects/Scene/ITMRepresentationAccess.h"

namespace ITMLib
{

//#################### HELPERS ####################

/**
 * \brief Calculates the bounds within an index image within which to draw a projected surfel.
 *
 * Note that a circular surfel should really project to an ellipse, but we use a circle for simplicity.
 * The difference in rendering quality this makes looks relatively minor in the literature.
 *
 * \param locId                   The raster position of the pixel in the index image to which the surfel's centre projects.
 * \param indexImageWidth         The width of the index image.
 * \param indexImageHeight        The height of the index image.
 * \param intrinsics              The intrinsic parameters of the depth camera.
 * \param radius                  The radius of the surfel.
 * \param z                       The depth value of the surfel's centre in live 3D depth coordinates.
 * \param cx                      The x coordinate of the pixel in the index image to which the surfel's centre projects.
 * \param cy                      The y coordinate of the pixel in the index image to which the surfel's centre projects.
 * \param projectedRadiusSquared  The square of the radius of the circle to use to represent the projected surfel in the index image.
 * \param minX                    The lower x bound of a bounding box around the circle in the index image (clamped to the image bounds).
 * \param minY                    The lower y bound of a bounding box around the circle in the index image (clamped to the image bounds).
 * \param maxX                    The upper x bound of a bounding box around the circle in the index image (clamped to the image bounds).
 * \param maxY                    The upper y bound of a bounding box around the circle in the index image (clamped to the image bounds).
 */
_CPU_AND_GPU_CODE_
inline void calculate_projected_surfel_bounds(int locId, int indexImageWidth, int indexImageHeight, const ITMIntrinsics& intrinsics, float radius, float z,
                                              int& cx, int& cy, int& projectedRadiusSquared, int& minX, int& minY, int& maxX, int& maxY)
{
  // Calculate the (x,y) coordinates of the pixel in the index image to which the surfel's centre projects.
  cx = locId % indexImageWidth, cy = locId / indexImageWidth;

  // Calculate the square of the radius of the circle to use to represent the projected surfel in the index image.
  float f = 0.5f * (intrinsics.projectionParamsSimple.fx + intrinsics.projectionParamsSimple.fy);
  int projectedRadius = static_cast<int>(radius * f / z + 0.5f);
  projectedRadiusSquared = projectedRadius * projectedRadius;

  // Calculate the lower and upper bounds of a bounding box around the circle in the index image.
  minX = cx - projectedRadius, maxX = cx + projectedRadius;
  minY = cy - projectedRadius, maxY = cy + projectedRadius;

  // Clamp these bounds to the image bounds.
  if(minX < 0) minX = 0;
  if(maxX >= indexImageWidth) maxX = indexImageWidth - 1;
  if(minY < 0) minY = 0;
  if(maxY >= indexImageHeight) maxY = indexImageHeight - 1;
}

/**
 * \brief Computes a colour to represent the specified normal vector.
 *
 * \param n The normal vector.
 * \return  A colour representation of the normal vector.
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

/**
 * \brief Projects a surfel into an index image.
 *
 * \param surfel            The surfel to project.
 * \param invT              A transformation mapping global coordinates to live 3D depth coordinates.
 * \param intrinsics        The intrinsic parameters of the depth camera.
 * \param indexImageWidth   The width of the index image.
 * \param indexImageHeight  The height of the index image.
 * \param scaleFactor       The scale factor by which the index image is supersampled with respect to the depth image.
 * \param locId             The raster position of the pixel in the index image to which the surfel's centre projects.
 * \param z                 The depth value of the surfel's centre in live 3D depth coordinates.
 * \param scaledZ           An integer representation of the depth value for storage in the depth buffer.
 * \return                  true, if the surfel projected to a pixel within the bounds of the index image, or false otherwise.
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

  // Convert the projected point into index image coordinates.
  int x = static_cast<int>(ux * scaleFactor + 0.5f);
  int y = static_cast<int>(uy * scaleFactor + 0.5f);

  // If the resulting point is outside the index image, early out.
  if(x < 0 || x >= indexImageWidth || y < 0 || y >= indexImageHeight) return false;

  // Calculate the raster position of the point in the index image.
  locId = y * indexImageWidth + x;

  // Calculate an integer depth value for the point for storage in the depth buffer.
  scaledZ = static_cast<int>(z * 100000);

  return true;
}

//#################### MAIN FUNCTIONS ####################

/**
 * \brief Clears the specified pixel of the surfel index image and the corresponding pixel in its depth buffer.
 *
 * \param locId             The raster position of the pixel in the index image.
 * \param surfelIndexImage  The index image.
 * \param depthBuffer       The depth buffer for the index image.
 */
_CPU_AND_GPU_CODE_
inline void clear_surfel_index_image(int locId, unsigned int *surfelIndexImage, int *depthBuffer)
{
  surfelIndexImage[locId] = 0;
  depthBuffer[locId] = INT_MAX;
}

/**
 * \brief Copies a surfel's correspondence information into buffers in order to support correspondence debugging using OpenGL.
 *
 * \param surfelId        The ID of the surfel.
 * \param surfels         The surfels in the scene.
 * \param newPositions    The buffer into which to store the "new" position of the surfel from its most recent merge.
 * \param oldPositions    The buffer into which to store the "old" position of the surfel from its most recent merge.
 * \param correspondences The buffer into which to store the "new" and "old" positions of the surfel for the purpose of rendering a line segment between them.
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
 * \brief Copies a surfel's position and normal into buffers that will be used for ICP tracking.
 *
 * \param locId                       The raster position of the surfel in the index image.
 * \param surfels                     The surfels in the scene.
 * \param surfelIndexImage            The index image.
 * \param invT                        A transformation mapping global coordinates to live 3D depth coordinates.
 * \param trackingSurfelMaxDepth      The maximum depth a surfel must have in order for it to be used for tracking.
 * \param trackingSurfelMinConfidence The minimum confidence value a surfel must have in order for it to be used for tracking.
 * \param pointsMap                   A buffer into which to write the surfel's position.
 * \param normalsMap                  A buffer into which to write the surfel's normal.
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
inline void copy_surfel_data_to_icp_maps(int locId, const TSurfel *surfels, const unsigned int *surfelIndexImage, const Matrix4f& invT,
                                         float trackingSurfelMaxDepth, float trackingSurfelMinConfidence, Vector4f *pointsMap, Vector4f *normalsMap)
{
  int surfelIndex = surfelIndexImage[locId] - 1;

  // If the specified raster position in the index image refers to a valid surfel:
  if(surfelIndex >= 0)
  {
    TSurfel surfel = surfels[surfelIndex];
    const Vector3f& p = surfel.position;
    const Vector3f& n = surfel.normal;

    // If the surfel is sufficiently close to the camera and has a sufficiently high confidence value:
    Vector3f v = transform_point(invT, p);
    if(v.z <= trackingSurfelMaxDepth || surfel.confidence >= trackingSurfelMinConfidence)
    {
      // Write the surfel's position and normal into the buffers.
      pointsMap[locId] = Vector4f(p.x, p.y, p.z, 1.0f);
      normalsMap[locId] = Vector4f(n.x, n.y, n.z, 0.0f);
      return;
    }
  }

  // Otherwise, write a dummy position and normal into the buffers.
  Vector4f dummy;
  dummy.x = dummy.y = dummy.z = 0.0f; dummy.w = -1.0f;
  pointsMap[locId] = dummy;
  normalsMap[locId] = dummy;
}

/**
 * \brief Copies a surfel's information into property-specific buffers (these can be used for rendering the surfel scene using OpenGL).
 *
 * \param surfelId  The ID of the surfel whose information is to be copied.
 * \param surfels   The surfels in the scene.
 * \param positions A buffer into which to write the surfel's position.
 * \param normals   A buffer into which to write the surfel's normal.
 * \param colours   A buffer into which to write the surfel's colour.
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
 * \brief Writes the colour of the surfel at a particular raster position in the index image to an output image.
 *
 * \param locId             The raster position in the index image.
 * \param surfelIndexImage  The index image.
 * \param surfels           The surfels in the scene.
 * \param outputImage       The output image.
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
 * \brief Writes (a colourised version of) the confidence value of the surfel at a particular raster position in the index image to an output image.
 *
 * \param locId                   The raster position in the index image.
 * \param surfelIndexImage        The index image.
 * \param surfels                 The surfels in the scene.
 * \param stableSurfelConfidence  The confidence value a surfel must have in order for it to be considered "stable".
 * \param outputImage             The output image.
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

    // Colourise the surfel's confidence value (red = unstable, green = stable).
    uchar g = (uchar)(255.0f * confidence / stableSurfelConfidence);
    col4 = Vector4u(255 - g, g, 0, 255);
  }

  outputImage[locId] = col4;
}

/**
 * \brief Writes the distance from a camera position of the surfel at a particular raster position in the index image to an output image.
 *
 * \param locId             The raster position in the index image.
 * \param surfelIndexImage  The index image.
 * \param surfels           The surfels in the scene.
 * \param cameraPosition    The camera position.
 * \param outputImage       The output image.
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
    float dx = fabs(cameraPosition.x - p.x);
    float dy = fabs(cameraPosition.y - p.y);
    float dz = fabs(cameraPosition.z - p.z);
    value = sqrt(dx * dx + dy * dy + dz * dz);
  }

  outputImage[locId] = value;
}

/**
 * \brief Writes a value for the light intensity at the surfel at a particular raster position in the index image to an output image.
 *
 * \param locId             The raster position in the index image.
 * \param surfelIndexImage  The index image.
 * \param surfels           The surfels in the scene.
 * \param lightPos          The position of the light (in global coordinates).
 * \param viewerPos         The position of the viewer (in global coordinates).
 * \param lightingType      The type of lighting model to use (e.g. Lambertian or Phong).
 * \param outputImage       The output image.
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_
void shade_pixel_grey(int locId, const unsigned int *surfelIndexImage, const TSurfel *surfels, const Vector3f& lightPos, const Vector3f& viewerPos,
                      SurfelLightingType lightingType, Vector4u *outputImage)
{
  const float ambient = lightingType == SLT_PHONG ? 0.3f : 0.2f;
  const float lambertianCoefficient = lightingType == SLT_PHONG ? 0.35f : 0.8f;
  const float phongCoefficient = 0.35f;
  const float phongExponent = 20.0f;

  Vector4u value((uchar)0);

  int surfelIndex = surfelIndexImage[locId] - 1;
  if(surfelIndex >= 0)
  {
    TSurfel surfel = surfels[surfelIndex];

    // Calculate the Lambertian lighting term.
    Vector3f L = normalize(lightPos - surfel.position);
    Vector3f N = surfel.normal;
    float NdotL = ORUtils::dot(N, L);
    float lambertian = CLAMP(NdotL, 0.0f, 1.0f);

    // Determine the intensity of the pixel using the Lambertian lighting equation.
    float intensity = lightingType != SLT_FLAT ? ambient + lambertianCoefficient * lambertian : 1.0f;

    // If we're using Phong lighting:
    if(lightingType == SLT_PHONG)
    {
      // Calculate the Phong lighting term.
      Vector3f R = 2.0f * N * NdotL - L;
      Vector3f V = normalize(viewerPos - surfel.position);
      float phong = pow(CLAMP(dot(R,V), 0.0f, 1.0f), phongExponent);

      // Add the Phong lighting term to the intensity.
      intensity += phongCoefficient * phong;
    }

    // Fill in the final value for the pixel.
    value = Vector4u((uchar)(intensity * 255.0f));
  }

  outputImage[locId] = value;
}

/**
 * \brief Writes (a colourised version of) the normal of the surfel at a particular raster position in the index image to an output image.
 *
 * \param locId             The raster position in the index image.
 * \param surfelIndexImage  The index image.
 * \param surfels           The surfels in the scene.
 * \param outputImage       The output image.
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
 * \brief Updates the depth buffer for an index image by projecting a surfel into the coordinate system of the index image and
 *        writing its depth value into the depth buffer if it is closer than the current nearest surfel along its ray.
 *
 * Note that this does not change the index image itself, it merely fills in the depth buffer so that the index image can then
 * be efficiently updated in parallel.
 *
 * \param surfelId                    The ID of the surfel being projected.
 * \param surfels                     The surfels in the scene.
 * \param invT                        A transformation mapping global coordinates to live 3D depth coordinates.
 * \param intrinsics                  The intrinsic parameters of the depth camera.
 * \param indexImageWidth             The width of the index image.
 * \param indexImageHeight            The height of the index image.
 * \param scaleFactor                 The scale factor by which the index image is supersampled with respect to the depth image.
 * \param useRadii                    Whether or not to render each surfel as a circle rather than a point.
 * \param unstableSurfelRenderingMode Whether to always/never render unstable surfels, or render them only if there's no stable alternative.
 * \param stableSurfelConfidence      The confidence value a surfel must have in order for it to be considered "stable".
 * \param unstableSurfelZOffset       The z offset to apply to unstable surfels when trying to ensure that they are only rendered if there is no stable alternative.
 * \param depthBuffer                 The depth buffer for the index image.
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void update_depth_buffer_for_surfel(int surfelId, const TSurfel *surfels, const Matrix4f& invT, const ITMIntrinsics& intrinsics,
                                           int indexImageWidth, int indexImageHeight, int scaleFactor, bool useRadii,
                                           UnstableSurfelRenderingMode unstableSurfelRenderingMode, float stableSurfelConfidence,
                                           int unstableSurfelZOffset, int *depthBuffer)
{
  const TSurfel surfel = surfels[surfelId];

  // Check whether the surfel is unstable. If it is, and we're not rendering unstable surfels, early out.
  bool unstableSurfel = surfel.confidence < stableSurfelConfidence;
  if(unstableSurfel && unstableSurfelRenderingMode == USR_DONOTRENDER) return;

  // If the projection of the surfel falls within the bounds of the index image:
  int locId, scaledZ;
  float z;
  if(project_surfel_to_index_image(surfel, invT, intrinsics, indexImageWidth, indexImageHeight, scaleFactor, locId, z, scaledZ))
  {
    // If the surfel's unstable and we're giving preference to stable surfels, add a z offset to ensure that
    // it will only be rendered if there's no stable alternative along the same ray.
    if(unstableSurfel && unstableSurfelRenderingMode == USR_FAUTEDEMIEUX) scaledZ += unstableSurfelZOffset;

    if(useRadii)
    {
      // If we're rendering the surfel as a circle rather than a point, calculate the radius of the projected
      // surfel and its bounds within the index image.
      int cx, cy, minX, minY, maxX, maxY, projectedRadiusSquared;
      calculate_projected_surfel_bounds(
        locId, indexImageWidth, indexImageHeight, intrinsics, surfel.radius, z,
        cx, cy, projectedRadiusSquared, minX, minY, maxX, maxY
      );

      // Rasterise the circle into the depth buffer.
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
      // If we're rendering the surfel as a point, simply update the corresponding pixel in the depth buffer as necessary.
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
 * \brief Updates an index image by rendering a surfel into it (using depth testing based on a pre-computed depth buffer).
 *
 * \param surfelId                    The ID of the surfel being rendered.
 * \param surfels                     The surfels in the scene.
 * \param invT                        A transformation mapping global coordinates to live 3D depth coordinates.
 * \param intrinsics                  The intrinsic parameters of the depth camera.
 * \param indexImageWidth             The width of the index image.
 * \param indexImageHeight            The height of the index image.
 * \param scaleFactor                 The scale factor by which the index image is supersampled with respect to the depth image.
 * \param depthBuffer                 The depth buffer for the index image.
 * \param useRadii                    Whether or not to render each surfel as a circle rather than a point.
 * \param unstableSurfelRenderingMode Whether to always/never render unstable surfels, or render them only if there's no stable alternative.
 * \param stableSurfelConfidence      The confidence value a surfel must have in order for it to be considered "stable".
 * \param unstableSurfelZOffset       The z offset to apply to unstable surfels when trying to ensure that they are only rendered if there is no stable alternative.
 * \param surfelIndexImage            The index image.
 */
template <typename TSurfel>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void update_index_image_for_surfel(int surfelId, const TSurfel *surfels, const Matrix4f& invT, const ITMIntrinsics& intrinsics,
                                          int indexImageWidth, int indexImageHeight, int scaleFactor, const int *depthBuffer, bool useRadii,
                                          UnstableSurfelRenderingMode unstableSurfelRenderingMode, float stableSurfelConfidence,
                                          int unstableSurfelZOffset, unsigned int *surfelIndexImage)
{
  const TSurfel surfel = surfels[surfelId];

  // Check whether the surfel is unstable. If it is, and we're not rendering unstable surfels, early out.
  bool unstableSurfel = surfel.confidence < stableSurfelConfidence;
  if(unstableSurfel && unstableSurfelRenderingMode == USR_DONOTRENDER) return;

  // If the projection of the surfel falls within the bounds of the index image:
  int locId, scaledZ;
  float z;
  if(project_surfel_to_index_image(surfel, invT, intrinsics, indexImageWidth, indexImageHeight, scaleFactor, locId, z, scaledZ))
  {
    // If the surfel's unstable and we're giving preference to stable surfels, add a z offset to ensure that
    // it will only be rendered if there's no stable alternative along the same ray.
    if(unstableSurfel && unstableSurfelRenderingMode == USR_FAUTEDEMIEUX) scaledZ += unstableSurfelZOffset;

    unsigned int surfelIdPlusOne = static_cast<unsigned int>(surfelId + 1);

    if(useRadii)
    {
      // If we're rendering the surfel as a circle rather than a point, calculate the radius of the projected
      // surfel and its bounds within the index image.
      int cx, cy, minX, minY, maxX, maxY, projectedRadiusSquared;
      calculate_projected_surfel_bounds(
        locId, indexImageWidth, indexImageHeight, intrinsics, surfel.radius, z,
        cx, cy, projectedRadiusSquared, minX, minY, maxX, maxY
      );

      // Rasterise the circle into the index image, taking account of the depths in the pre-computed depth buffer.
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
      // If we're rendering the surfel as a point, simply update the corresponding pixel in the index image as necessary.
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
