// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#pragma once

namespace ITMLib
{

/**
 * \brief The different types of lighting that can be used for surfel visualisation.
 *
 * FIXME: This is very similar to the LightingType enum in spaint - they should be combined one day.
 */
enum SurfelLightingType
{
  SLT_FLAT,
  SLT_LAMBERTIAN,
  SLT_PHONG
};

/**
 * \brief Whether or not to render unstable surfels.
 */
enum UnstableSurfelRenderingMode
{
  /** Do not render unstable surfels. */
  USR_DONOTRENDER,

  /** Render unstable surfels only if there is no stable surfel along the same ray. */
  USR_FAUTEDEMIEUX,

  /** Render unstable surfels even if there is a stable surfel along the same ray. */
  USR_RENDER,
};

}
