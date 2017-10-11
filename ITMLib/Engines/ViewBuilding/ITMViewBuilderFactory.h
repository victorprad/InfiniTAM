// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "Interface/ITMViewBuilder.h"
#include "../../Utils/ITMLibSettings.h"

namespace ITMLib
{

/**
 * \brief This struct provides functions that can be used to construct view builders.
 */
struct ITMViewBuilderFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a view builder.
   *
   * \param calib       The joint RGBD calibration parameters.
   * \param deviceType  The device on which the view builder should operate.
   */
  static ITMViewBuilder *MakeViewBuilder(const ITMRGBDCalib& calib, ITMLibSettings::DeviceType deviceType);
};

}
