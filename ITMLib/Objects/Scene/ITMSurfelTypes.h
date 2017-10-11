// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#pragma once

#include "../../Utils/ITMMath.h"

#define DEBUG_CORRESPONDENCES 0

namespace ITMLib
{
  /**
   * \brief An instance of this struct represents a surfel that does not contain colour information.
   */
  struct ITMSurfel_grey
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

#if DEBUG_CORRESPONDENCES
    /** The new position of the surfel (prior to fusing). */
    Vector3f newPosition;

    /** The old position of the surfel (prior to fusing). */
    Vector3f oldPosition;
#endif
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

#if DEBUG_CORRESPONDENCES
    /** The new position of the surfel (prior to fusing). */
    Vector3f newPosition;

    /** The old position of the surfel (prior to fusing). */
    Vector3f oldPosition;
#endif
  };
}
