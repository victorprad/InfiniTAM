// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdexcept>

#include "ITMLowLevelEngine.h"
#include "ITMTracker.h"
#include "DeviceSpecific/CPU/ITMRenTracker_CPU.h"
#include "DeviceSpecific/CUDA/ITMRenTracker_CUDA.h"
#include "../Objects/ITMTrackingState.h"
#include "../Utils/ITMLibSettings.h"

namespace ITMLib
{
  namespace Engine
  {
    namespace ITMTrackerFactory_NS
    {
      using namespace ITMLib::Objects;

      /**
       * \brief This factory can be used to construct trackers and tracking states based on a specified set of InfiniTAM settings.
       */
      class ITMTrackerFactory
      {
      private:
        /** \brief Prevent instantiation. */
        ITMTrackerFactory();

      public:
        /**
         * \brief Makes a primary tracker based on the specified InfiniTAM settings.
         *
         * Note: The returned pointer points to memory allocated with new. Clients should call delete to deallocate it once it is no longer needed.
         *
         * \param settings        The settings controlling which tracker to use.
         * \param imgSize_rgb     The dimensions of the RGB frames we're using.
         * \param imgSize_d       The dimensions of the depth frames we're using.
         * \param lowLevelEngine  A pointer to the low-level image processing engine we're using.
         * \return                A pointer to the new tracker.
         */
        static ITMTracker *MakePrimaryTracker(const ITMLibSettings& settings, const Vector2i& imgSize_rgb, const Vector2i& imgSize_d, ITMLowLevelEngine *lowLevelEngine);

        /**
         * \brief Makes a secondary tracker based on the specified InfiniTAM settings.
         *
         * Note 1: A secondary tracker is only needed in the case of Ren tracking - when using other trackers, the function will return NULL.
         * Note 2: The returned pointer points to memory allocated with new. Clients should call delete to deallocate it once it is no longer needed.
         *
         * \param settings        The settings controlling which tracker to use.
         * \param imgSize_rgb     The dimensions of the RGB frames we're using.
         * \param imgSize_d       The dimensions of the depth frames we're using.
         * \param lowLevelEngine  A pointer to the low-level image processing engine we're using.
         * \param scene           A pointer to the scene.
         * \return                A pointer to the new tracker, if any, or NULL otherwise.
         */
        template <typename TVoxel, typename TIndex>
        static ITMTracker *MakeSecondaryTracker(const ITMLibSettings& settings, const Vector2i& imgSize_rgb, const Vector2i& imgSize_d, ITMLowLevelEngine *lowLevelEngine, ITMScene<TVoxel,TIndex> *scene)
        {
          if(settings.useGPU)
          {
#ifndef COMPILE_WITHOUT_CUDA
            return settings.trackerType == ITMLibSettings::TRACKER_REN ? new ITMRenTracker_CUDA<TVoxel,TIndex>(imgSize_d, settings.noICPRunTillLevel, lowLevelEngine, scene) : NULL;
#else
            // This should never happen.
            throw std::runtime_error("Error: ITMTrackerFactory::MakeSecondaryTracker: CUDA support not currently available.");
#endif
          }
          else
          {
            return settings.trackerType == ITMLibSettings::TRACKER_REN ? new ITMRenTracker_CPU<TVoxel,TIndex>(imgSize_d, settings.noICPRunTillLevel, lowLevelEngine, scene) : NULL;
          }
        }

        /**
         * \brief Makes a tracking state suitable for use with the tracker(s) we want to use.
         *
         * Note: The returned pointer points to memory allocated with new. Clients should call delete to deallocate it once it is no longer needed.
         *
         * \param settings        The settings controlling which tracker to use.
         * \param imgSize_rgb     The dimensions of the RGB frames we're using.
         * \param imgSize_d       The dimensions of the depth frames we're using.
         * \return                A pointer to the new tracking state.
         */
        static ITMTrackingState *MakeTrackingState(const ITMLibSettings& settings, const Vector2i& imgSize_rgb, const Vector2i& imgSize_d);
      };
    }

    using ITMTrackerFactory_NS::ITMTrackerFactory;
  }
}
