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
	/* This helpful abstract interface alows you to ignore the fact that
	   scenes are templates.
	*/
	class ITMLocalSceneManager
	{
		public:
		virtual ~ITMLocalSceneManager(void) {}

		virtual int createNewScene(void) = 0;
		virtual void removeScene(int index) = 0;
		virtual size_t numScenes(void) const = 0;

		virtual void addRelation(int fromScene, int toScene, const ITMPose & pose, int weight) = 0;
		virtual bool resetTracking(int sceneID, const ITMPose & pose) = 0;

		virtual const ITMPose* getTrackingPose(int sceneID) const = 0;
		virtual int getSceneSize(int sceneID) const = 0;
	};

	template<class TVoxel, class TIndex>
	class ITMLocalSceneManager_instance : public ITMLocalSceneManager
	{
		private:
		const ITMLibSettings *settings;
		const IITMVisualisationEngine *visualisationEngine;
		const ITMTrackingController *trackingController;
		const ITMDenseMapper<TVoxel,TIndex> *denseMapper;
		Vector2i trackedImageSize;

		std::vector<ITMLocalScene<TVoxel,TIndex>*> allData;

		public:
		ITMLocalSceneManager_instance(const ITMLibSettings *settings, const IITMVisualisationEngine *visualisationEngine, const ITMTrackingController *trackingController, const ITMDenseMapper<TVoxel,TIndex> *denseMapper, const Vector2i & trackedImageSize);
		~ITMLocalSceneManager_instance(void);

		int createNewScene(void);
		void removeScene(int index);
		size_t numScenes(void) const
		{ return allData.size(); }

		void addRelation(int fromScene, int toScene, const ITMPose & pose, int weight);
		bool resetTracking(int sceneID, const ITMPose & pose);

		const ITMLocalScene<TVoxel,TIndex>* getScene(int sceneID) const
		{ return allData[sceneID]; }

		ITMLocalScene<TVoxel,TIndex>* getScene(int sceneID)
		{ return allData[sceneID]; }

		const ITMPose* getTrackingPose(int sceneID) const
		{ return getScene(sceneID)->trackingState->pose_d; }
		int getSceneSize(int sceneID) const;

		std::vector<int> getShortestLinkPath(int fromSceneID, int toSceneID) const;
		ITMPose findTransformation(int fromSceneID, int toSceneID) const;
	};

	/** \brief
	*/
	class ITMActiveSceneManager
	{
	public:
		typedef enum { PRIMARY_SCENE, NEW_SCENE, LOOP_CLOSURE, RELOCALISATION, LOST, LOST_NEW } SceneActivity;

	private:
		struct ActiveDataDescriptor {
			int sceneIndex;
			SceneActivity type;
			std::vector<Matrix4f> constraints;
			ITMPose estimatedPose;
			int trackingAttempts;
		};

		ITMLocalSceneManager *localSceneManager;
		std::vector<ActiveDataDescriptor> activeData;

		int CheckSuccess_relocalisation(int dataID) const;
		int CheckSuccess_newlink(int dataID, int *inliers, ITMPose *inlierPose) const;
		void AcceptNewLink(int dataId, int primaryDataId, const ITMPose & pose, int weight);

	public:
		bool shouldStartNewArea(void) const;
		bool shouldMovePrimaryScene(int newDataIdx, int bestDataIdx, int primaryDataIdx) const;

		void initiateNewScene(bool isPrimaryScene = false);
		int initiateNewLink(int sceneID, const ITMPose & pose, bool isRelocalisation);

		void recordTrackingResult(int dataID, int trackingSuccess, bool primaryTrackingSuccess);
		void maintainActiveData(void);

		int findPrimaryDataIdx(void) const;
		int findPrimarySceneIdx(void) const;

		int numActiveScenes(void) const
		{ return activeData.size(); }
		int getSceneIndex(int dataIdx) const
		{ return activeData[dataIdx].sceneIndex; }
		SceneActivity getSceneType(int dataIdx) const
		{ return activeData[dataIdx].type; }

		ITMActiveSceneManager(ITMLocalSceneManager *localSceneManager);
		~ITMActiveSceneManager(void) {}
	};
}

