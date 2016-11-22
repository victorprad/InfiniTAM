// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMMainEngine.h"
#include "ITMTrackingController.h"
#include "../Engines/LowLevel/Interface/ITMLowLevelEngine.h"
#include "../Engines/ViewBuilding/Interface/ITMViewBuilder.h"
#include "../Objects/Misc/ITMIMUCalibrator.h"
#include "../../RelocLib/Relocaliser.h"
#include "../../RelocLib/PoseDatabase.h"

#include "../Engines/MultiScene/ITMActiveMapManager.h"
#include "../Engines/MultiScene/ITMGlobalAdjustmentEngine.h"
#include "../Engines/Visualisation/Interface/ITMMultiVisualisationEngine.h"

#include <vector>

namespace ITMLib
{
	/** \brief
	*/
	template <typename TVoxel, typename TIndex>
	class ITMMultiEngine : public ITMMainEngine
	{
	private:
		const ITMLibSettings *settings;

		ITMLowLevelEngine *lowLevelEngine;
		ITMVisualisationEngine<TVoxel, TIndex> *visualisationEngine;
		ITMMultiVisualisationEngine<TVoxel, TIndex> *multiVisualisationEngine;

		ITMViewBuilder *viewBuilder;
		ITMTrackingController *trackingController;
		ITMTracker *tracker;
		ITMIMUCalibrator *imuCalibrator;
		ITMDenseMapper<TVoxel, TIndex> *denseMapper;

		RelocLib::Relocaliser *mLoopClosureDetector;
		RelocLib::PoseDatabase mPoseDatabase;

		ITMVoxelMapGraphManager<TVoxel, TIndex> *mSceneManager;
		ITMActiveMapManager *mActiveDataManager;
		ITMGlobalAdjustmentEngine *mGlobalAdjustmentEngine;
		bool mScheduleGlobalAdjustment;

		Vector2i trackedImageSize;
		ITMRenderState *renderState_freeview;
		ITMRenderState *renderState_multiscene;
		int freeviewSceneIdx;

		/// Pointer for storing the current input frame
		ITMView *view;
	public:
		ITMView* GetView() { return view; }

		ITMTrackingState* GetTrackingState(void);

		/// Process a frame with rgb and depth images and (optionally) a corresponding imu measurement
		ITMTrackingState::TrackingResult ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement = NULL);

		/// Get a result image as output
		Vector2i GetImageSize(void) const;

		void GetImage(ITMUChar4Image *out, GetImageType getImageType, ORUtils::SE3Pose *pose = NULL, ITMIntrinsics *intrinsics = NULL);

		void changeFreeviewSceneIdx(ORUtils::SE3Pose *pose, int newIdx);
		void setFreeviewSceneIdx(int newIdx)
		{
			freeviewSceneIdx = newIdx;
		}
		int getFreeviewSceneIdx(void) const
		{
			return freeviewSceneIdx;
		}
		int findPrimarySceneIdx(void) const
		{
			return mActiveDataManager->findPrimarySceneIdx();
		}

		//void writeFullTrajectory(void) const;
		//void SaveSceneToMesh(const char *objFileName);

		/** \brief Constructor
			Ommitting a separate image size for the depth images
			will assume same resolution as for the RGB images.
		*/
		ITMMultiEngine(const ITMLibSettings *settings, const ITMRGBDCalib &calib, Vector2i imgSize_rgb, Vector2i imgSize_d = Vector2i(-1, -1));
		~ITMMultiEngine(void);
	};
}