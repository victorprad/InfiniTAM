// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdexcept>
#include <vector>

#include "CPU/ITMColorTracker_CPU.h"
#include "CPU/ITMDepthTracker_CPU.h"
#include "CPU/ITMExtendedTracker_CPU.h"
#include "Interface/ITMCompositeTracker.h"
#include "Interface/ITMIMUTracker.h"
#include "Interface/ITMFileBasedTracker.h"
#include "Interface/ITMForceFailTracker.h"
#include "Interface/ITMTracker.h"
#include "../Engines/LowLevel/Interface/ITMLowLevelEngine.h"
#include "../Utils/ITMLibSettings.h"

#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/ITMColorTracker_CUDA.h"
#include "CUDA/ITMDepthTracker_CUDA.h"
#include "CUDA/ITMExtendedTracker_CUDA.h"
#endif

#ifdef COMPILE_WITH_METAL
#include "Metal/ITMExtendedTracker_Metal.h"
#endif

#include "../../ORUtils/KeyValueConfig.h"

namespace ITMLib
{
	/**
	 * \brief An instance of this class can be used to construct trackers.
	 */
	class ITMTrackerFactory
	{
	private:
		//#################### TYPEDEFS ####################
		typedef ITMTracker* MakerFunc(const Vector2i&,const Vector2i&,ITMLibSettings::DeviceType,const ORUtils::KeyValueConfig &,const ITMLowLevelEngine*,ITMIMUCalibrator*,const ITMSceneParams*);

		/// Tracker types
		typedef enum {
			//! Identifies a tracker based on colour image
			TRACKER_COLOR,
			//! Identifies a tracker based on depth image
			TRACKER_ICP,
			//! Identifies a tracker based on depth and color image with various extensions
			TRACKER_EXTENDED,
			//! Identifies a tracker reading poses from text files
			TRACKER_FILE,
			//! Identifies a tracker based on depth image and IMU measurement
			TRACKER_IMU,
			//! Identifies a tracker based on depth and colour images and IMU measurement
			TRACKER_EXTENDEDIMU,
			//! Identifies a tracker that forces tracking to fail
			TRACKER_FORCEFAIL,
		} TrackerType;

		struct Maker {
			const char *id;
			const char *description;
			TrackerType type;
			MakerFunc *make;

			Maker(const char *_id, const char *_desc, TrackerType _type, MakerFunc *_make)
			  : id(_id), description(_desc), type(_type), make(_make)
			{}
		};

		//#################### PRIVATE VARIABLES ####################
		/** A list of maker functions for the various tracker types. */
		std::vector<Maker> makers;

		//################## SINGLETON IMPLEMENTATION ##################
		/**
		 * \brief Constructs a tracker factory.
		 */
		ITMTrackerFactory(void)
		{
			makers.push_back(Maker("rgb", "Colour based tracker", TRACKER_COLOR, &MakeColourTracker));
			makers.push_back(Maker("icp", "Depth based ICP tracker", TRACKER_ICP, &MakeICPTracker));
			makers.push_back(Maker("extended", "Depth + colour based tracker", TRACKER_EXTENDED, &MakeExtendedTracker));
			makers.push_back(Maker("file", "File based tracker", TRACKER_FILE, &MakeFileBasedTracker));
			makers.push_back(Maker("imuicp", "Combined IMU and depth based ICP tracker", TRACKER_IMU, &MakeIMUTracker));
			makers.push_back(Maker("extendedimu", "Combined IMU and depth + colour ICP tracker", TRACKER_EXTENDEDIMU, &MakeExtendedIMUTracker));
			makers.push_back(Maker("forcefail", "Force fail tracker", TRACKER_FORCEFAIL, &MakeForceFailTracker));
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

		//################## PUBLIC MEMBER FUNCTIONS ##################
	public:
	/**
	 * \brief Makes a tracker of the type specified in the trackerConfig string.
	 */
	ITMTracker *Make(ITMLibSettings::DeviceType deviceType, const char *trackerConfig, const Vector2i & imgSize_rgb, const Vector2i & imgSize_d, const ITMLowLevelEngine *lowLevelEngine,
		ITMIMUCalibrator *imuCalibrator, const ITMSceneParams *sceneParams) const
	{
		ORUtils::KeyValueConfig cfg(trackerConfig);
		int verbose = 0;
		if (cfg.getProperty("help") != NULL) if (verbose < 10) verbose = 10;

		ORUtils::KeyValueConfig::ChoiceList trackerOptions;
		for (int i = 0; (unsigned)i < makers.size(); ++i) {
			trackerOptions.addChoice(makers[i].id, makers[i].type);
		}
		int type = TRACKER_ICP;
		cfg.parseChoiceProperty("type", "type of tracker", type, trackerOptions, verbose);
		const Maker *maker = NULL;
		for (int i = 0; (unsigned)i < makers.size(); ++i) {
			if (makers[i].type == type) {
				maker = &(makers[i]);
				break;
			}
		}
		if (maker == NULL) DIEWITHEXCEPTION("Unknown tracker type");

		ITMTracker *ret = (*(maker->make))(imgSize_rgb, imgSize_d, deviceType, cfg, lowLevelEngine, imuCalibrator, sceneParams);
		if (ret->requiresColourRendering()) {
			printf("Assuming a voxel type with colour information!");
		}

		return ret;
	}

	/**
	 * \brief Makes a tracker of the type specified in the settings.
	 */
	ITMTracker *Make(const Vector2i & imgSize_rgb, const Vector2i & imgSize_d, const ITMLibSettings *settings, const ITMLowLevelEngine *lowLevelEngine,
		ITMIMUCalibrator *imuCalibrator, const ITMSceneParams *sceneParams) const
	{
		return Make(settings->deviceType, settings->trackerConfig, imgSize_rgb, imgSize_d, lowLevelEngine, imuCalibrator, sceneParams);
	}

	//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
	static std::vector<TrackerIterationType> parseLevelConfig(const char *str)
	{
		bool parseError = false;
		std::vector<TrackerIterationType> ret;
		for (int i = static_cast<int>(strlen(str))-1; i >= 0; --i) {
			switch (str[i]) {
			case 'r':
				ret.push_back(TRACKER_ITERATION_ROTATION);
				break;
			case 't':
				ret.push_back(TRACKER_ITERATION_TRANSLATION);
				break;
			case 'b':
				ret.push_back(TRACKER_ITERATION_BOTH);
				break;
			case 'n':
				ret.push_back(TRACKER_ITERATION_NONE);
				break;
			default:
				parseError = true;
				break;
			}
		}

		if (parseError) {
			fprintf(stderr, "error parsing level configuration '%s'\n", str);
			for (int i = 0; (unsigned)i < ret.size(); ++i) fprintf(stderr, "level %i: %i\n", (int)ret.size()-i, (int)(ret[ret.size()-i]));
		}
		return ret;
	}

	/**
	 * \brief Makes a colour tracker.
	 */
	static ITMTracker *MakeColourTracker(const Vector2i& imgSize_rgb, const Vector2i& imgSize_d, ITMLibSettings::DeviceType deviceType, const ORUtils::KeyValueConfig & cfg,
		const ITMLowLevelEngine *lowLevelEngine, ITMIMUCalibrator *imuCalibrator, const ITMSceneParams *sceneParams)
	{
		int verbose = 0;
		if (cfg.getProperty("help") != NULL) if (verbose < 10) verbose = 10;

		const char *levelSetup = "rrrbb";
		cfg.parseStrProperty("levels", "resolution hierarchy levels", levelSetup, verbose);
		std::vector<TrackerIterationType> levels = parseLevelConfig(levelSetup);

		ITMColorTracker *ret = NULL;
		switch (deviceType)
		{
		case ITMLibSettings::DEVICE_CPU:
			ret = new ITMColorTracker_CPU(imgSize_rgb, &(levels[0]), static_cast<int>(levels.size()), lowLevelEngine);
			break;
		case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
			ret = new ITMColorTracker_CUDA(imgSize_rgb, &(levels[0]), static_cast<int>(levels.size()), lowLevelEngine);
#endif
			break;
		case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
			ret = new ITMColorTracker_CPU(imgSize_rgb, &(levels[0]), static_cast<int>(levels.size()), lowLevelEngine);
#endif
			break;
		}

		if (ret==NULL) DIEWITHEXCEPTION("Failed to make colour tracker");
		return ret;
	}

	/**
	 * \brief Makes an ICP tracker.
	 */
	static ITMTracker *MakeICPTracker(const Vector2i& imgSize_rgb, const Vector2i& imgSize_d, ITMLibSettings::DeviceType deviceType, const ORUtils::KeyValueConfig & cfg,
		const ITMLowLevelEngine *lowLevelEngine, ITMIMUCalibrator *imuCalibrator, const ITMSceneParams *sceneParams)
	{
		const char *levelSetup = "rrrbb";
		float smallStepSizeCriterion = 1e-3f;
		float outlierDistanceFine = 0.002f;
		float outlierDistanceCoarse = 0.01f;
		float failureDetectorThd = 3.0f;
		int numIterationsCoarse = 10;
		int numIterationsFine = 2;

		int verbose = 0;
		if (cfg.getProperty("help") != NULL) if (verbose < 10) verbose = 10;
		cfg.parseStrProperty("levels", "resolution hierarchy levels", levelSetup, verbose);
		std::vector<TrackerIterationType> levels = parseLevelConfig(levelSetup);

		cfg.parseFltProperty("minstep", "step size threshold for convergence", smallStepSizeCriterion, verbose);
		cfg.parseFltProperty("outlierC", "outlier threshold at coarsest level", outlierDistanceCoarse, verbose);
		cfg.parseFltProperty("outlierF", "outlier threshold at finest level", outlierDistanceFine, verbose);
		cfg.parseIntProperty("numiterC", "maximum number of iterations at coarsest level", numIterationsCoarse, verbose);
		cfg.parseIntProperty("numiterF", "maximum number of iterations at finest level", numIterationsFine, verbose);
		cfg.parseFltProperty("failureDec", "threshold for the failure detection", failureDetectorThd, verbose);

		ITMDepthTracker *ret = NULL;
		switch (deviceType)
		{
		case ITMLibSettings::DEVICE_CPU:
			ret = new ITMDepthTracker_CPU(imgSize_d, &(levels[0]), static_cast<int>(levels.size()), smallStepSizeCriterion, failureDetectorThd, lowLevelEngine);
			break;
		case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
			ret = new ITMDepthTracker_CUDA(imgSize_d, &(levels[0]), static_cast<int>(levels.size()), smallStepSizeCriterion, failureDetectorThd, lowLevelEngine);
#endif
			break;
		case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
			ret = new ITMDepthTracker_CPU(imgSize_d, &(levels[0]), static_cast<int>(levels.size()), smallStepSizeCriterion, failureDetectorThd, lowLevelEngine);
#endif
			break;
		}

		if (ret == NULL) DIEWITHEXCEPTION("Failed to make ICP tracker");
		ret->SetupLevels(numIterationsCoarse, numIterationsFine,
			outlierDistanceCoarse, outlierDistanceFine);
		return ret;
	}

	/**
	* \brief Makes an Extended tracker.
	*/
	static ITMTracker *MakeExtendedTracker(const Vector2i& imgSize_rgb, const Vector2i& imgSize_d, ITMLibSettings::DeviceType deviceType, const ORUtils::KeyValueConfig & cfg,
		const ITMLowLevelEngine *lowLevelEngine, ITMIMUCalibrator *imuCalibrator, const ITMSceneParams *sceneParams)
	{
		const char *levelSetup = "rrbb";
		bool useDepth = true;
		bool useColour = false;
		float colourWeight = 0.3f;
		float smallStepSizeCriterion = 1e-4f;
		float outlierSpaceDistanceFine = 0.004f;
		float outlierSpaceDistanceCoarse = 0.1f;
		float outlierColourDistanceFine = 0.175f;
		float outlierColourDistanceCoarse = 0.005f;
		float failureDetectorThd = 3.0f;
		float minColourGradient = 0.01f;
		float tukeyCutOff = 8.0f;
		int framesToSkip = 20;
		int framesToWeight = 50;
		int numIterationsCoarse = 20;
		int numIterationsFine = 20;

		int verbose = 0;
		if (cfg.getProperty("help") != NULL) if (verbose < 10) verbose = 10;
		cfg.parseStrProperty("levels", "resolution hierarchy levels", levelSetup, verbose);
		std::vector<TrackerIterationType> levels = parseLevelConfig(levelSetup);

		cfg.parseBoolProperty("useDepth", "use ICP based tracking", useDepth, verbose);
		cfg.parseBoolProperty("useColour", "use colour based tracking", useColour, verbose);
		cfg.parseFltProperty("colourWeight", "weight used to scale colour errors and jacobians when both useColour and useWeights are set", colourWeight, verbose);
		cfg.parseFltProperty("minstep", "step size threshold for convergence", smallStepSizeCriterion, verbose);
		cfg.parseFltProperty("outlierSpaceC", "space outlier threshold at coarsest level", outlierSpaceDistanceCoarse, verbose);
		cfg.parseFltProperty("outlierSpaceF", "space outlier threshold at finest level", outlierSpaceDistanceFine, verbose);
		cfg.parseFltProperty("outlierColourC", "colour outlier threshold at coarsest level", outlierColourDistanceCoarse, verbose);
		cfg.parseFltProperty("outlierColourF", "colour outlier threshold at finest level", outlierColourDistanceFine, verbose);
		cfg.parseFltProperty("minColourGradient", "minimum colour gradient for a pixel to be used in the tracking", minColourGradient, verbose);
		cfg.parseIntProperty("numiterC", "maximum number of iterations at coarsest level", numIterationsCoarse, verbose);
		cfg.parseIntProperty("numiterF", "maximum number of iterations at finest level", numIterationsFine, verbose);
		cfg.parseFltProperty("tukeyCutOff", "cutoff for the tukey m-estimator", tukeyCutOff, verbose);
		cfg.parseIntProperty("framesToSkip", "number of frames to skip before depth pixel is used for tracking", framesToSkip, verbose);
		cfg.parseIntProperty("framesToWeight", "number of frames to weight each depth pixel for before using it fully", framesToWeight, verbose);
		cfg.parseFltProperty("failureDec", "threshold for the failure detection", failureDetectorThd, verbose);

		ITMExtendedTracker *ret = NULL;
		switch (deviceType)
		{
		case ITMLibSettings::DEVICE_CPU:
			ret = new ITMExtendedTracker_CPU(imgSize_d,
											 imgSize_rgb,
											 useDepth,
											 useColour,
											 colourWeight,
											 &(levels[0]),
											 static_cast<int>(levels.size()),
											 smallStepSizeCriterion,
											 failureDetectorThd,
											 sceneParams->viewFrustum_min,
											 sceneParams->viewFrustum_max,
											 minColourGradient,
											 tukeyCutOff,
											 framesToSkip,
											 framesToWeight,
											 lowLevelEngine);
			break;
		case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
			ret = new ITMExtendedTracker_CUDA(imgSize_d,
												imgSize_rgb,
												useDepth,
												useColour,
												colourWeight,
												&(levels[0]),
												static_cast<int>(levels.size()),
												smallStepSizeCriterion,
												failureDetectorThd,
												sceneParams->viewFrustum_min,
												sceneParams->viewFrustum_max,
												minColourGradient,
												tukeyCutOff,
												framesToSkip,
												framesToWeight,
												lowLevelEngine);
#endif
			break;
		case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
			ret = new ITMExtendedTracker_Metal(imgSize_d, imgSize_rgb, useDepth, useColour, colourWeight, &(levels[0]), static_cast<int>(levels.size()), smallStepSizeCriterion, failureDetectorThd,
				scene->sceneParams->viewFrustum_min, scene->sceneParams->viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight, lowLevelEngine);
#endif
			break;
		}

		if (ret == NULL) DIEWITHEXCEPTION("Failed to make extended tracker");
		ret->SetupLevels(numIterationsCoarse, numIterationsFine, outlierSpaceDistanceCoarse, outlierSpaceDistanceFine, outlierColourDistanceCoarse, outlierColourDistanceFine);
		return ret;
	}

	/**
	 * \brief Makes an IMU tracker.
	 */
	static ITMTracker* MakeIMUTracker(const Vector2i& imgSize_rgb, const Vector2i& imgSize_d, ITMLibSettings::DeviceType deviceType, const ORUtils::KeyValueConfig & cfg,
		const ITMLowLevelEngine *lowLevelEngine, ITMIMUCalibrator *imuCalibrator, const ITMSceneParams *sceneParams)
	{
		const char *levelSetup = "tb";
		float smallStepSizeCriterion = 1e-3f;
		float outlierDistanceFine = 0.005f;
		float outlierDistanceCoarse = 0.01f;
		float failureDetectorThd = 3.0f;
		int numIterationsCoarse = 4;
		int numIterationsFine = 2;

		int verbose = 0;
		if (cfg.getProperty("help") != NULL) if (verbose < 10) verbose = 10;
		cfg.parseStrProperty("levels", "resolution hierarchy levels", levelSetup, verbose);
		std::vector<TrackerIterationType> levels = parseLevelConfig(levelSetup);

		cfg.parseFltProperty("minstep", "step size threshold for convergence", smallStepSizeCriterion, verbose);
		cfg.parseFltProperty("outlierC", "outlier threshold at coarsest level", outlierDistanceCoarse, verbose);
		cfg.parseFltProperty("outlierF", "outlier threshold at finest level", outlierDistanceFine, verbose);
		cfg.parseIntProperty("numiterC", "maximum number of iterations at coarsest level", numIterationsCoarse, verbose);
		cfg.parseIntProperty("numiterF", "maximum number of iterations at finest level", numIterationsFine, verbose);
		cfg.parseFltProperty("failureDec", "threshold for the failure detection", failureDetectorThd, verbose);

		ITMDepthTracker *dTracker = NULL;
		switch (deviceType)
		{
		case ITMLibSettings::DEVICE_CPU:
			dTracker = new ITMDepthTracker_CPU(imgSize_d, &(levels[0]), static_cast<int>(levels.size()), smallStepSizeCriterion, failureDetectorThd, lowLevelEngine);
			break;
		case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
			dTracker = new ITMDepthTracker_CUDA(imgSize_d, &(levels[0]), static_cast<int>(levels.size()), smallStepSizeCriterion, failureDetectorThd, lowLevelEngine);
#endif
			break;
		case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
			dTracker = new ITMDepthTracker_CPU(imgSize_d, &(levels[0]), static_cast<int>(levels.size()), smallStepSizeCriterion, failureDetectorThd, lowLevelEngine);
#endif
			break;
		default: break;
		}

		if (dTracker == NULL) DIEWITHEXCEPTION("Failed to make IMU tracker");
		dTracker->SetupLevels(numIterationsCoarse, numIterationsFine,
			outlierDistanceCoarse, outlierDistanceFine);

		ITMCompositeTracker *compositeTracker = new ITMCompositeTracker;
		compositeTracker->AddTracker(new ITMIMUTracker(imuCalibrator));
		compositeTracker->AddTracker(dTracker);
		return compositeTracker;
	}

	/**
	* \brief Makes an Extended IMU tracker.
	*/
	static ITMTracker* MakeExtendedIMUTracker(const Vector2i& imgSize_rgb, const Vector2i& imgSize_d, ITMLibSettings::DeviceType deviceType, const ORUtils::KeyValueConfig & cfg,
		const ITMLowLevelEngine *lowLevelEngine, ITMIMUCalibrator *imuCalibrator, const ITMSceneParams *sceneParams)
	{
		ITMTracker *dTracker = MakeExtendedTracker(imgSize_rgb, imgSize_d, deviceType, cfg,
				lowLevelEngine, imuCalibrator, sceneParams);
		if (dTracker == NULL) DIEWITHEXCEPTION("Failed to make extended tracker"); // Should never happen though

		ITMCompositeTracker *compositeTracker = new ITMCompositeTracker;
		compositeTracker->AddTracker(new ITMIMUTracker(imuCalibrator));
		compositeTracker->AddTracker(dTracker);
		return compositeTracker;
	}

	/**
	 * \brief Makes a file based tracker.
	 */
	static ITMTracker *MakeFileBasedTracker(const Vector2i& imgSize_rgb, const Vector2i& imgSize_d, ITMLibSettings::DeviceType deviceType, const ORUtils::KeyValueConfig & cfg,
		const ITMLowLevelEngine *lowLevelEngine, ITMIMUCalibrator *imuCalibrator, const ITMSceneParams *sceneParams)
	{
		int verbose = 0;
		if (cfg.getProperty("help") && verbose < 10) verbose = 10;

		const char *fileMask = "";
		int initialFrameNo = 0;
		cfg.parseStrProperty("mask", "mask for the saved pose text files", fileMask, verbose);
		cfg.parseIntProperty("initialFrameNo", "initial frame index to use for tracking", initialFrameNo, verbose);

		return new ITMFileBasedTracker(fileMask, initialFrameNo);
	}

	/**
	 * \brief Makes a force fail tracker.
	 */
	static ITMTracker *MakeForceFailTracker(const Vector2i& imgSize_rgb, const Vector2i& imgSize_d, ITMLibSettings::DeviceType deviceType, const ORUtils::KeyValueConfig & cfg,
		const ITMLowLevelEngine *lowLevelEngine, ITMIMUCalibrator *imuCalibrator, const ITMSceneParams *sceneParams)
	{
		return new ITMForceFailTracker;
	}
};
}
