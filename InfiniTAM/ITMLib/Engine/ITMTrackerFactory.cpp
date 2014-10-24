// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMTrackerFactory.h"

#include "DeviceSpecific/CPU/ITMColorTracker_CPU.h"
#include "DeviceSpecific/CUDA/ITMColorTracker_CUDA.h"
#include "DeviceSpecific/CPU/ITMDepthTracker_CPU.h"
#include "DeviceSpecific/CUDA/ITMDepthTracker_CUDA.h"
using namespace ITMLib::Engine;

ITMTracker *ITMTrackerFactory::MakePrimaryTracker(const ITMLibSettings& settings, const Vector2i& imgSize_rgb, const Vector2i& imgSize_d, ITMLowLevelEngine *lowLevelEngine)
{
  if(settings.useGPU)
  {
#ifndef COMPILE_WITHOUT_CUDA
    switch(settings.trackerType)
    {
    case ITMLibSettings::TRACKER_ICP:
    case ITMLibSettings::TRACKER_REN:
      return new ITMDepthTracker_CUDA(imgSize_d, settings.noHierarchyLevels, settings.noRotationOnlyLevels, settings.noICPRunTillLevel, settings.depthTrackerICPThreshold, lowLevelEngine);
    case ITMLibSettings::TRACKER_COLOR:
      return new ITMColorTracker_CUDA(imgSize_rgb, settings.noHierarchyLevels, settings.noRotationOnlyLevels, lowLevelEngine);
    default:
      throw std::runtime_error("Error: ITMTrackerFactory::MakePrimaryTracker: Unknown tracker type");
    }
#else
    // This should never happen.
    throw std::runtime_error("Error: ITMTrackerFactory::MakePrimaryTracker: CUDA support not currently available.");
#endif
  }
  else
  {
    switch(settings.trackerType)
    {
    case ITMLibSettings::TRACKER_ICP:
    case ITMLibSettings::TRACKER_REN:
      return new ITMDepthTracker_CPU(imgSize_d, settings.noHierarchyLevels, settings.noRotationOnlyLevels, settings.noICPRunTillLevel, settings.depthTrackerICPThreshold, lowLevelEngine);
    case ITMLibSettings::TRACKER_COLOR:
      return new ITMColorTracker_CPU(imgSize_rgb, settings.noHierarchyLevels, settings.noRotationOnlyLevels, lowLevelEngine);
    default:
      throw std::runtime_error("Error: ITMTrackerFactory::MakePrimaryTracker: Unknown tracker type");
    }
  }
}

ITMTrackingState *ITMTrackerFactory::MakeTrackingState(const ITMLibSettings& settings, const Vector2i& imgSize_rgb, const Vector2i& imgSize_d)
{
  return new ITMTrackingState(settings.trackerType == ITMLibSettings::TRACKER_COLOR ? imgSize_rgb : imgSize_d, settings.useGPU);
}
