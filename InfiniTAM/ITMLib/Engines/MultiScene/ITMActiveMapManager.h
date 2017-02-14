// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMMapGraphManager.h"

namespace ITMLib
{
	/** \brief
	*/
	class ITMActiveMapManager
	{
	public:
		typedef enum { PRIMARY_LOCAL_MAP, NEW_LOCAL_MAP, LOOP_CLOSURE, RELOCALISATION, LOST, LOST_NEW } LocalMapActivity;

	private:
		struct ActiveDataDescriptor 
		{
			int localMapIndex;
			LocalMapActivity type;
			std::vector<Matrix4f> constraints;
			ORUtils::SE3Pose estimatedPose;
			int trackingAttempts;
		};

		ITMMapGraphManager *localMapManager;
		std::vector<ActiveDataDescriptor> activeData;

		int CheckSuccess_relocalisation(int dataID) const;
		int CheckSuccess_newlink(int dataID, int primaryDataID, int *inliers, ORUtils::SE3Pose *inlierPose) const;
		void AcceptNewLink(int dataId, int primaryDataId, const ORUtils::SE3Pose & pose, int weight);

		float visibleOriginalBlocks(int dataID) const;
		bool shouldStartNewArea(void) const;
		bool shouldMovePrimaryLocalMap(int newDataIdx, int bestDataIdx, int primaryDataIdx) const;

	public:
		int initiateNewLocalMap(bool isPrimaryLocalMap = false);
		int initiateNewLink(int sceneID, const ORUtils::SE3Pose & pose, bool isRelocalisation);

		void recordTrackingResult(int dataID, ITMTrackingState::TrackingResult trackingResult, bool primaryTrackingSuccess);
		
		// return whether or not the local map graph has changed
		bool maintainActiveData(void);

		int findPrimaryDataIdx(void) const;
		int findPrimaryLocalMapIdx(void) const;

		int findBestVisualisationDataIdx(void) const;
		int findBestVisualisationLocalMapIdx(void) const;

		int numActiveLocalMaps(void) const { return static_cast<int>(activeData.size()); }
		int getLocalMapIndex(int dataIdx) const { return activeData[dataIdx].localMapIndex; }
		LocalMapActivity getLocalMapType(int dataIdx) const { return activeData[dataIdx].type; }

		ITMActiveMapManager(ITMMapGraphManager *localMapManager);
		~ITMActiveMapManager(void) {}
	};
}