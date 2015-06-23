// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <map>
#include <stdexcept>

#include "ITMCompositeTracker.h"
#include "ITMIMUTracker.h"
#include "ITMLowLevelEngine.h"
#include "ITMTracker.h"

#include "DeviceSpecific/CPU/ITMColorTracker_CPU.h"
#include "DeviceSpecific/CPU/ITMDepthTracker_CPU.h"
#include "DeviceSpecific/CPU/ITMWeightedICPTracker_CPU.h"
#include "DeviceSpecific/CPU/ITMRenTracker_CPU.h"
#include "../Utils/ITMLibSettings.h"

#ifndef COMPILE_WITHOUT_CUDA
#include "DeviceSpecific/CUDA/ITMColorTracker_CUDA.h"
#include "DeviceSpecific/CUDA/ITMDepthTracker_CUDA.h"
#include "DeviceSpecific/CUDA/ITMWeightedICPTracker_CUDA.h"
#include "DeviceSpecific/CUDA/ITMRenTracker_CUDA.h"
#endif

#ifdef COMPILE_WITH_METAL
#include "DeviceSpecific/Metal/ITMDepthTracker_Metal.h"
#endif

namespace ITMLib
{
  namespace Engine
  {
    /**
     * \brief An instance of this class can be used to construct trackers.
     */
    template <typename TVoxel, typename TIndex>
    class ITMTrackerFactory
    {
      //#################### TYPEDEFS ####################
    private:
      typedef ITMTracker *(*Maker)(const Vector2i&,const ITMLibSettings*,const ITMLowLevelEngine*,ITMIMUCalibrator*,ITMScene<TVoxel,TIndex>*);

      //#################### PRIVATE VARIABLES ####################
    private:
      /** A map of maker functions for the various different tracker types. */
      std::map<ITMLibSettings::TrackerType,Maker> makers;

      //#################### SINGLETON IMPLEMENTATION ####################
    private:
      /**
       * \brief Constructs a tracker factory.
       */
      ITMTrackerFactory()
      {
        makers.insert(std::make_pair(ITMLibSettings::TRACKER_COLOR, &MakeColourTracker));
        makers.insert(std::make_pair(ITMLibSettings::TRACKER_ICP, &MakeICPTracker));
		makers.insert(std::make_pair(ITMLibSettings::TRACKER_WICP, &MakeWeightedICPTracker));
        makers.insert(std::make_pair(ITMLibSettings::TRACKER_IMU, &MakeIMUTracker));
        makers.insert(std::make_pair(ITMLibSettings::TRACKER_REN, &MakeRenTracker));
      }

    public:
      /**
       * \brief Gets the singleton instance for the current set of template parameters.
       */
      static ITMTrackerFactory& Instance()
      {
        static ITMTrackerFactory s_instance;
        return s_instance;
      }

      //#################### PUBLIC MEMBER FUNCTIONS ####################
    public:
      /**
       * \brief Makes a tracker of the type specified in the settings.
       */
      ITMTracker *Make(const Vector2i& trackedImageSize, const ITMLibSettings *settings, const ITMLowLevelEngine *lowLevelEngine,
                       ITMIMUCalibrator *imuCalibrator, ITMScene<TVoxel,TIndex> *scene) const
      {
        typename std::map<ITMLibSettings::TrackerType,Maker>::const_iterator it = makers.find(settings->trackerType);
        if(it == makers.end()) DIEWITHEXCEPTION("Unknown tracker type");

        Maker maker = it->second;
        return (*maker)(trackedImageSize, settings, lowLevelEngine, imuCalibrator, scene);
      }

      //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
    public:
      /**
       * \brief Makes a colour tracker.
       */
      static ITMTracker *MakeColourTracker(const Vector2i& trackedImageSize, const ITMLibSettings *settings, const ITMLowLevelEngine *lowLevelEngine,
                                           ITMIMUCalibrator *imuCalibrator, ITMScene<TVoxel,TIndex> *scene)
      {
        switch(settings->deviceType)
        {
          case ITMLibSettings::DEVICE_CPU:
          {
            return new ITMColorTracker_CPU(trackedImageSize, settings->trackingRegime, settings->noHierarchyLevels, lowLevelEngine);
          }
          case ITMLibSettings::DEVICE_CUDA:
          {
#ifndef COMPILE_WITHOUT_CUDA
            return new ITMColorTracker_CUDA(trackedImageSize, settings->trackingRegime, settings->noHierarchyLevels, lowLevelEngine);
#else
            break;
#endif
          }
          case ITMLibSettings::DEVICE_METAL:
          {
#ifdef COMPILE_WITH_METAL
            return new ITMColorTracker_CPU(trackedImageSize, settings->trackingRegime, settings->noHierarchyLevels, lowLevelEngine);
#else
            break;
#endif
          }
          default: break;
        }

        DIEWITHEXCEPTION("Failed to make colour tracker");
      }

      /**
       * \brief Makes an ICP tracker.
       */
      static ITMTracker *MakeICPTracker(const Vector2i& trackedImageSize, const ITMLibSettings *settings, const ITMLowLevelEngine *lowLevelEngine,
                                        ITMIMUCalibrator *imuCalibrator, ITMScene<TVoxel,TIndex> *scene)
      {
        switch(settings->deviceType)
        {
          case ITMLibSettings::DEVICE_CPU:
          {
            return new ITMDepthTracker_CPU(
              trackedImageSize,
              settings->trackingRegime,
              settings->noHierarchyLevels,
              settings->noICPRunTillLevel,
              settings->depthTrackerICPThreshold,
              settings->depthTrackerTerminationThreshold,
              lowLevelEngine
            );
          }
          case ITMLibSettings::DEVICE_CUDA:
          {
#ifndef COMPILE_WITHOUT_CUDA
            return new ITMDepthTracker_CUDA(
              trackedImageSize,
              settings->trackingRegime,
              settings->noHierarchyLevels,
              settings->noICPRunTillLevel,
              settings->depthTrackerICPThreshold,
              settings->depthTrackerTerminationThreshold,
              lowLevelEngine
            );
#else
            break;
#endif
          }
          case ITMLibSettings::DEVICE_METAL:
          {
#ifdef COMPILE_WITH_METAL
            return new ITMDepthTracker_Metal(
              trackedImageSize,
              settings->trackingRegime,
              settings->noHierarchyLevels,
              settings->noICPRunTillLevel,
              settings->depthTrackerICPThreshold,
              settings->depthTrackerTerminationThreshold,
              lowLevelEngine
            );
#else
            break;
#endif
          }
          default: break;
        }

        DIEWITHEXCEPTION("Failed to make ICP tracker");
      }
	  /**
	  * \brief Makes an WICP tracker.
	  */
	  static ITMTracker *MakeWeightedICPTracker(const Vector2i& trackedImageSize, const ITMLibSettings *settings, const ITMLowLevelEngine *lowLevelEngine,
		  ITMIMUCalibrator *imuCalibrator, ITMScene<TVoxel, TIndex> *scene)
	  {
		  switch (settings->deviceType)
		  {
		  case ITMLibSettings::DEVICE_CPU:
		  {
			  return new ITMWeightedICPTracker_CPU(
				  trackedImageSize,
				  settings->trackingRegime,
				  settings->noHierarchyLevels,
				  settings->noICPRunTillLevel,
				  settings->depthTrackerICPThreshold,
				  settings->depthTrackerTerminationThreshold,
				  lowLevelEngine
				  );
		  }
		  case ITMLibSettings::DEVICE_CUDA:
		  {
#ifndef COMPILE_WITHOUT_CUDA
			  return new ITMWeightedICPTracker_CUDA(
				  trackedImageSize,
				  settings->trackingRegime,
				  settings->noHierarchyLevels,
				  settings->noICPRunTillLevel,
				  settings->depthTrackerICPThreshold,
				  settings->depthTrackerTerminationThreshold,
				  lowLevelEngine
				  );
#else
			  break;
#endif
		  }
		  case ITMLibSettings::DEVICE_METAL:
		  {
#ifdef COMPILE_WITH_METAL
			  return new ITMDepthTracker_Metal(
				  trackedImageSize,
				  settings->trackingRegime,
				  settings->noHierarchyLevels,
				  settings->noICPRunTillLevel,
				  settings->depthTrackerICPThreshold,
				  settings->depthTrackerTerminationThreshold,
				  lowLevelEngine
				  );
#else
			  break;
#endif
		  }
		  default: break;
		  }

		  throw std::runtime_error("Failed to make ICP tracker");
	  }

      /**
       * \brief Makes an IMU tracker.
       */
      static ITMTracker *MakeIMUTracker(const Vector2i& trackedImageSize, const ITMLibSettings *settings, const ITMLowLevelEngine *lowLevelEngine,
                                        ITMIMUCalibrator *imuCalibrator, ITMScene<TVoxel,TIndex> *scene)
      {
        switch(settings->deviceType)
        {
          case ITMLibSettings::DEVICE_CPU:
          {
            ITMCompositeTracker *compositeTracker = new ITMCompositeTracker(2);
            compositeTracker->SetTracker(new ITMIMUTracker(imuCalibrator), 0);
            compositeTracker->SetTracker(
              new ITMDepthTracker_CPU(
                trackedImageSize,
                settings->trackingRegime,
                settings->noHierarchyLevels,
                settings->noICPRunTillLevel,
                settings->depthTrackerICPThreshold,
                settings->depthTrackerTerminationThreshold,
                lowLevelEngine
              ), 1
            );
            return compositeTracker;
          }
          case ITMLibSettings::DEVICE_CUDA:
          {
#ifndef COMPILE_WITHOUT_CUDA
            ITMCompositeTracker *compositeTracker = new ITMCompositeTracker(2);
            compositeTracker->SetTracker(new ITMIMUTracker(imuCalibrator), 0);
            compositeTracker->SetTracker(
              new ITMDepthTracker_CUDA(
                trackedImageSize,
                settings->trackingRegime,
                settings->noHierarchyLevels,
                settings->noICPRunTillLevel,
                settings->depthTrackerICPThreshold,
                settings->depthTrackerTerminationThreshold,
                lowLevelEngine
              ), 1
            );
            return compositeTracker;
#else
            break;
#endif
          }
          case ITMLibSettings::DEVICE_METAL:
          {
#ifdef COMPILE_WITH_METAL
            ITMCompositeTracker *compositeTracker = new ITMCompositeTracker(2);
            compositeTracker->SetTracker(new ITMIMUTracker(imuCalibrator), 0);
            compositeTracker->SetTracker(
              new ITMDepthTracker_Metal(
                trackedImageSize,
                settings->trackingRegime,
                settings->noHierarchyLevels,
                settings->noICPRunTillLevel,
                settings->depthTrackerICPThreshold,
                settings->depthTrackerTerminationThreshold,
                lowLevelEngine
              ), 1
            );
            return compositeTracker;
#else
            break;
#endif
          }
          default: break;
        }

        DIEWITHEXCEPTION("Failed to make IMU tracker");
      }

      /**
       * \brief Makes a Ren tracker.
       */
      static ITMTracker *MakeRenTracker(const Vector2i& trackedImageSize, const ITMLibSettings *settings, const ITMLowLevelEngine *lowLevelEngine,
                                        ITMIMUCalibrator *imuCalibrator, ITMScene<TVoxel,TIndex> *scene)
      {
        switch(settings->deviceType)
        {
          case ITMLibSettings::DEVICE_CPU:
          {
            return new ITMRenTracker_CPU<TVoxel, TIndex>(
              trackedImageSize,
              settings->trackingRegime,
              2,
              lowLevelEngine, scene
            );
          }
          case ITMLibSettings::DEVICE_CUDA:
          {
#ifndef COMPILE_WITHOUT_CUDA
            return new ITMRenTracker_CUDA<TVoxel, TIndex>(
              trackedImageSize,
              settings->trackingRegime,
              2,
              lowLevelEngine,
              scene
            );
#else
            break;
#endif
          }
          case ITMLibSettings::DEVICE_METAL:
          {
#ifdef COMPILE_WITH_METAL
            return new ITMRenTracker_CPU<TVoxel, TIndex>(
              trackedImageSize,
              settings->trackingRegime,
              2,
              lowLevelEngine, scene
            );
#else
            break;
#endif
          }
          default: break;
        }

        DIEWITHEXCEPTION("Failed to make Ren tracker");
      }
    };
  }
}
