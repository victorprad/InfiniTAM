// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

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

}
