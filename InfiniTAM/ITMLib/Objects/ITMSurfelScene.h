// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMMath.h"

namespace ITMLib
{
  /**
   * \brief An instance of this struct represents a surface element, or "surfel", in a 3D scene.
   */
  struct ITMSurfel
  {
    //#################### PUBLIC VARIABLES ####################

    /** The confidence counter for the surfel. */
    float confidence;

    /** The surface normal at the surfel. */
    Vector3f normal;

    /** The position of the surfel. */
    Vector3f position;

    /** The radius of the surfel. */
    float radius;

    /** A timestamp for the surfel, recording the last frame in which it was updated. */
    int timestamp;
  };

  /**
   * \brief An instance of an instantiation of this class template represents a surfel-based scene.
   */
  template <typename TSurfel>
  class ITMSurfelScene
  {
    //#################### PRIVATE VARIABLES ####################
  private:
    // TODO

    //#################### CONSTRUCTORS ####################
  public:
    // TODO

    //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
  private:
    // Deliberately private and unimplemented
    ITMSurfelScene(const ITMSurfelScene&);
    ITMSurfelScene& operator=(const ITMSurfelScene&);

    //#################### PUBLIC MEMBER FUNCTIONS ####################
  public:
    // TODO
  };
}
