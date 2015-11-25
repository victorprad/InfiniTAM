// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMMath.h"

namespace ITMLib
{
  /**
   * \brief An instance of this struct represents a surfel that does not contain colour information.
   */
  struct ITMSurfel
  {
    static const CONSTPTR(bool) hasColourInformation = false;

    // Note: The ordering of the variables here matters because it affects padding - do not reorder without prior thought.

    /** The surface normal at the surfel. */
    Vector3f normal;

    /** The position of the surfel. */
    Vector3f position;

    /** The confidence counter for the surfel. */
    float confidence;

    /** The radius of the surfel. */
    float radius;

    /** A timestamp for the surfel, recording the last frame in which it was updated. */
    int timestamp;
  };

  /**
   * \brief An instance of this struct represents a surfel that contains colour information.
   */
  struct ITMSurfel_rgb
  {
    static const CONSTPTR(bool) hasColourInformation = true;

    // Note: The ordering of the variables here matters because it affects padding - do not reorder without prior thought.

    /** The RGB colour of the surfel. */
    Vector3u colour;

    /** The surface normal at the surfel. */
    Vector3f normal;

    /** The position of the surfel. */
    Vector3f position;

    /** The confidence counter for the surfel. */
    float confidence;

    /** The radius of the surfel. */
    float radius;

    /** A timestamp for the surfel, recording the last frame in which it was updated. */
    int timestamp;
  };

  /**
   * \brief TODO
   */
  template <bool hasColour> struct SurfelColourManipulator;

  /**
   * \brief TODO
   */
  template <>
  struct SurfelColourManipulator<false>
  {
    template <typename TSurfel>
    _CPU_AND_GPU_CODE_
    static Vector3u read(const TSurfel& surfel)
    {
      return Vector3u((uchar)0);
    }

    template <typename TSurfel>
    _CPU_AND_GPU_CODE_
    static void write(TSurfel& surfel, const Vector3u& colour)
    {
      // No-op
    }
  };

  /**
   * \brief TODO
   */
  template <>
  struct SurfelColourManipulator<true>
  {
    template <typename TSurfel>
    _CPU_AND_GPU_CODE_
    static Vector3u read(const TSurfel& surfel)
    {
      return surfel.colour;
    }

    template <typename TSurfel>
    _CPU_AND_GPU_CODE_
    static void write(TSurfel& surfel, const Vector3u& colour)
    {
      surfel.colour = colour;
    }
  };
}
