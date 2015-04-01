// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../ITMLib.h"
#include "ITMMainEngine.h"
#include "../Objects/ITMLocalScene.h"

#include <vector>

namespace ITMLib
{
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

		std::vector<ITMLocalScene<ITMVoxel,ITMVoxelIndex>*> allData;
		std::vector<std::map<int,ITMPoseConstraint> > allRelations;
		std::vector<int> activeDataIdx;
		int primaryDataIdx;

		Vector2i trackedImageSize;
		ITMRenderState *renderState_freeview;
		int freeviewDataIdx;

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
		void AddNewLocalScene(void);

		void ChangeFreeviewDataIdx(ITMPose *pose, int newIdx);
		void SetFreeviewDataIdx(int newIdx)
		{ freeviewDataIdx = newIdx; }
		int GetFreeviewDataIdx(void) const
		{ return freeviewDataIdx; }
		int GetPrimaryDataIdx(void) const
		{ return primaryDataIdx; }

		/** \brief Constructor
		    Ommitting a separate image size for the depth images
		    will assume same resolution as for the RGB images.
		*/
		ITMMultiEngine(const ITMLibSettings *settings, const ITMRGBDCalib *calib, Vector2i imgSize_rgb, Vector2i imgSize_d = Vector2i(-1,-1));
		~ITMMultiEngine();
	};
}

