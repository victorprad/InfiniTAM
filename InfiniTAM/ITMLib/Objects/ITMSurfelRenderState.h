// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ORUtils/Image.h"

namespace ITMLib
{
  /**
   * \brief TODO
   */
  struct ITMSurfelRenderState
  {
    /** TODO */
    ORUtils::Image<float> *depthBuffer;

    /** TODO */
    ORUtils::Image<unsigned long> *surfelIndexImage;
  };
}
