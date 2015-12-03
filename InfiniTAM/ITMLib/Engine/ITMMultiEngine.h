// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../ITMLib.h"
#include "ITMMainEngine.h"
#include "../Objects/ITMLocalScene.h"

#include "../../LCDLib/LoopClosureDetector.h"
#include "../../LCDLib/PoseDatabase.h"

#include <vector>

namespace ITMLib
{
	struct ActiveDataDescriptor {
		int sceneIndex;
		enum { PRIMARY_SCENE, NEW_SCENE, LOOP_CLOSURE, RELOCALISATION, LOST, LOST_NEW } type;
		std::vector<Matrix4f> constraints;
		ITMPose estimatedPose;
		int trackingAttempts;
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

		LCDLib::LoopClosureDetector *mLoopClosureDetector;
		LCDLib::PoseDatabase poseDatabase;

		std::vector<ITMLocalScene<ITMVoxel,ITMVoxelIndex>*> allData;
		std::vector<ActiveDataDescriptor> activeData;

		Vector2i trackedImageSize;
		ITMRenderState *renderState_freeview;
		int freeviewDataIdx;

		/// Pointer for storing the current input frame
		ITMView *view;
	public:
		ITMView* GetView() { return view; }

		ITMTrackingState* GetTrackingState(void);

		/// Process a frame with rgb and depth images and (optionally) a corresponding imu measurement
		void ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement = NULL);

		/// Get a result image as output
		Vector2i GetImageSize(void) const;

		void GetImage(ITMUChar4Image *out, GetImageType getImageType, ITMPose *pose = NULL, ITMIntrinsics *intrinsics = NULL);

		bool shouldStartNewArea(void) const;
		bool shouldMovePrimaryScene(int newDataIdx, int bestDataIdx, int primaryDataIdx) const;
		void AddNewLocalScene(int primarySceneIdx);
		bool AddNewRelocalisationScene(int sceneID, int primarySceneIdx, const ITMPose & pose);
		int CheckSuccess_relocalisation(int dataID) const;
		int CheckSuccess_newlink(int dataID, int *inliers, ITMPose *inlierPose) const;
		void AcceptNewLink(int dataId, int primaryDataId, ITMPose pose, int weight);
		void DiscardLocalScene(int sceneID);
		static ITMPose EstimateRelativePose(const std::vector<Matrix4f> & observations, int *out_numInliers, ITMPose *out_inlierPose);

		void ChangeFreeviewDataIdx(ITMPose *pose, int newIdx);
		void SetFreeviewDataIdx(int newIdx)
		{ freeviewDataIdx = newIdx; }
		int GetFreeviewDataIdx(void) const
		{ return freeviewDataIdx; }
		int FindPrimaryDataIdx(void) const;

		/** \brief Constructor
		    Ommitting a separate image size for the depth images
		    will assume same resolution as for the RGB images.
		*/
		ITMMultiEngine(const ITMLibSettings *settings, const ITMRGBDCalib *calib, Vector2i imgSize_rgb, Vector2i imgSize_d = Vector2i(-1,-1));
		~ITMMultiEngine();
	};
}

