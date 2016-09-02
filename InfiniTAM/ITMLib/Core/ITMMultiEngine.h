// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMMainEngine.h"
#include "ITMTrackingController.h"
#include "../Engines/LowLevel/Interface/ITMLowLevelEngine.h"
#include "../Engines/ViewBuilding/Interface/ITMViewBuilder.h"
#include "../Objects/Misc/ITMIMUCalibrator.h"
#include "../../RelocLib/Relocaliser.h"
#include "../../RelocLib/PoseDatabase.h"

#include "MultiScene/ITMLocalScene.h"
#include "MultiScene/ITMActiveSceneManager.h"

#include <vector>

namespace ITMLib
{
/*	struct ActiveDataDescriptor {
		int sceneIndex;
		enum { PRIMARY_SCENE, NEW_SCENE, LOOP_CLOSURE, RELOCALISATION, LOST, LOST_NEW } type;
		std::vector<Matrix4f> constraints;
		ORUtils::SE3Pose estimatedPose;
		int trackingAttempts;
	};*/

	/** \brief
	*/
	template <typename TVoxel, typename TIndex>
	class ITMMultiEngine : public ITMMainEngine
	{
	private:
		const ITMLibSettings *settings;

		ITMLowLevelEngine *lowLevelEngine;
		ITMVisualisationEngine<TVoxel,TIndex> *visualisationEngine;

		ITMViewBuilder *viewBuilder;		
		ITMTrackingController *trackingController;
		ITMTracker *tracker;
		ITMIMUCalibrator *imuCalibrator;
		ITMDenseMapper<TVoxel,TIndex> *denseMapper;

		RelocLib::Relocaliser *mRelocaliser;
		RelocLib::PoseDatabase poseDatabase;

/*		std::vector<ITMLocalScene<ITMVoxel,ITMVoxelIndex>*> allData;
		std::vector<ActiveDataDescriptor> activeData;*/
		ITMMultiSceneManager_instance<TVoxel,TIndex> *sceneManager;
		ITMActiveSceneManager *activeDataManager;

		Vector2i trackedImageSize;
		ITMRenderState *renderState_freeview;
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

/*		bool shouldStartNewArea(void) const;
		bool shouldMovePrimaryScene(int newDataIdx, int bestDataIdx, int primaryDataIdx) const;
		void AddNewLocalScene(int primarySceneIdx);
		bool AddNewRelocalisationScene(int sceneID, int primarySceneIdx, const ORUtils::SE3Pose & pose);
		int CheckSuccess_relocalisation(int dataID) const;
		int CheckSuccess_newlink(int dataID, int *inliers, ORUtils::SE3Pose *inlierPose) const;
		void AcceptNewLink(int dataId, int primaryDataId, ORUtils::SE3Pose pose, int weight);
		void DiscardLocalScene(int sceneID);
		static ORUtils::SE3Pose EstimateRelativePose(const std::vector<Matrix4f> & observations, int *out_numInliers, ORUtils::SE3Pose *out_inlierPose);
*/
		void changeFreeviewSceneIdx(ORUtils::SE3Pose *pose, int newIdx);
		void setFreeviewSceneIdx(int newIdx)
		{ freeviewSceneIdx = newIdx; }
		int getFreeviewSceneIdx(void) const
		{ return freeviewSceneIdx; }
		int findPrimarySceneIdx(void) const
		{ return activeDataManager->findPrimarySceneIdx(); }

		/** \brief Constructor
		    Ommitting a separate image size for the depth images
		    will assume same resolution as for the RGB images.
		*/
		ITMMultiEngine(const ITMLibSettings *settings, const ITMRGBDCalib& calib, Vector2i imgSize_rgb, Vector2i imgSize_d = Vector2i(-1,-1));
		~ITMMultiEngine(void);
	};
}

