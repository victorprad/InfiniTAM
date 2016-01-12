// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "Interface/ITMSurfelVisualisationEngine.h"
#include "../../Utils/ITMLibSettings.h"

namespace ITMLib
{
  /**
   * \brief An instantiation of this struct can be used to construct surfel visualisation engines.
   */
  template <typename TSurfel>
  struct ITMSurfelVisualisationEngineFactory
  {
    //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

    /**
     * \brief Makes a surfel visualisation engine.
     *
     * \param deviceType  TODO
     * \return            TODO
     */
    static ITMSurfelVisualisationEngine<TSurfel> *make_surfel_visualisation_engine(ITMLib::ITMLibSettings::DeviceType deviceType);
  };
}
