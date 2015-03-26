// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../ITMLib.h"
#include "../Utils/ITMLibSettings.h"
#include "ITMMainEngine.h"

#include <vector>

namespace ITMLib
{
	template<class TVoxel,class TIndex>
	class ITMLocalModelData
	{
	public:
		ITMScene<TVoxel,TIndex> *scene;
		ITMRenderState *renderState;
		ITMTrackingState *trackingState;

		ITMLocalModelData(const ITMLibSettings *settings, const IITMVisualisationEngine *visualisationEngine, const ITMTrackingController *trackingController, const Vector2i & trackedImageSize)
		{
			scene = new ITMScene<TVoxel,TIndex>(&(settings->sceneParams), settings->useSwapping, settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU);
			renderState = visualisationEngine->CreateRenderState(scene, trackedImageSize);
			trackingState = trackingController->BuildTrackingState(trackedImageSize);
		}
		~ITMLocalModelData(void)
		{
			delete scene;
			delete renderState;
			delete trackingState;
		}

	/*	ITMScene<TVoxel,TIndex>* getScene(void) { return scene; }
		ITMRenderState* getRenderState(void) { return renderState; }
		ITMTrackingState* getTrackingState(void) { return trackingState; }*/
	};

	/** \brief
	*/
	class ITMMultiEngine : public ITMMainEngine
	{
	private:
		const ITMLibSettings *settings;

		ITMLowLevelEngine *lowLevelEngine;
		IITMVisualisationEngine *visualisationEngine;

		ITMViewBuilder *viewBuilder;		
		ITMTrackingController *trackingController;
		ITMTracker *tracker;
		ITMIMUCalibrator *imuCalibrator;
		ITMDenseMapper<ITMVoxel,ITMVoxelIndex> *denseMapper;

		std::vector<ITMLocalModelData<ITMVoxel,ITMVoxelIndex>*> allData;
		std::vector<int> activeDataIdx;
		int primaryDataIdx;

		Vector2i trackedImageSize;
		ITMRenderState *renderState_freeview;

		/// Pointer for storing the current input frame
		ITMView *view;
	public:
		ITMView* GetView() { return view; }

		ITMTrackingState* GetTrackingState(void) { return allData[primaryDataIdx]->trackingState; }

		/// Process a frame with rgb and depth images and (optionally) a corresponding imu measurement
		void ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement = NULL);

		/// Get a result image as output
		Vector2i GetImageSize(void) const;

		void GetImage(ITMUChar4Image *out, GetImageType getImageType, ITMPose *pose = NULL, ITMIntrinsics *intrinsics = NULL);

		bool shouldStartNewArea(void) const;

		/** \brief Constructor
		    Ommitting a separate image size for the depth images
		    will assume same resolution as for the RGB images.
		*/
		ITMMultiEngine(const ITMLibSettings *settings, const ITMRGBDCalib *calib, Vector2i imgSize_rgb, Vector2i imgSize_d = Vector2i(-1,-1));
		~ITMMultiEngine();
	};
}

