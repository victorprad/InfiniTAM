// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <vector>

#include "ITMDenseMapper.h"
#include "ITMLocalScene.h"
#include "../Engines/Visualisation/Interface/ITMVisualisationEngine.h"

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
		virtual void getRelation(int fromScene, int toScene, ITMPose *out_pose = NULL, int *out_weight = NULL) const = 0;
		virtual bool resetTracking(int sceneID, const ITMPose & pose) = 0;

		virtual const ITMPose* getTrackingPose(int sceneID) const = 0;
		virtual int getSceneSize(int sceneID) const = 0;
		virtual int countVisibleBlocks(int sceneID, int minBlockId, int maxBlockId, bool invertIDs) const = 0;
	};

	template<class TVoxel, class TIndex>
	class ITMLocalSceneManager_instance : public ITMLocalSceneManager
	{
		private:
		const ITMLibSettings *settings;
		const ITMVisualisationEngine<TVoxel,TIndex> *visualisationEngine;
		const ITMDenseMapper<TVoxel,TIndex> *denseMapper;
		Vector2i trackedImageSize;

		std::vector<ITMLocalScene<TVoxel,TIndex>*> allData;

		public:
		ITMLocalSceneManager_instance(const ITMLibSettings *settings, const ITMVisualisationEngine<TVoxel,TIndex> *visualisationEngine, const ITMDenseMapper<TVoxel,TIndex> *denseMapper, const Vector2i & trackedImageSize);
		~ITMLocalSceneManager_instance(void);

		int createNewScene(void);
		void removeScene(int index);
		size_t numScenes(void) const
		{ return allData.size(); }

		void addRelation(int fromScene, int toScene, const ITMPose & pose, int weight);
		void getRelation(int fromScene, int toScene, ITMPose *out_pose, int *out_weight) const;
		bool resetTracking(int sceneID, const ITMPose & pose);

		const ITMLocalScene<TVoxel,TIndex>* getScene(int sceneID) const
		{ return allData[sceneID]; }

		ITMLocalScene<TVoxel,TIndex>* getScene(int sceneID)
		{ return allData[sceneID]; }

		const ITMPose* getTrackingPose(int sceneID) const
		{ return getScene(sceneID)->trackingState->pose_d; }
		int getSceneSize(int sceneID) const;
		int countVisibleBlocks(int sceneID, int minBlockId, int maxBlockId, bool invertIDs) const;

		std::vector<int> getShortestLinkPath(int fromSceneID, int toSceneID) const;
		ITMPose findTransformation(int fromSceneID, int toSceneID) const;
	};
}
