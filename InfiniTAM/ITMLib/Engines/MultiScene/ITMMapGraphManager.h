// Copyright 2016 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <vector>

#include "../../Objects/Scene/ITMLocalMap.h"
#include "../../Core/ITMDenseMapper.h"
#include "../../Engines/Visualisation/Interface/ITMVisualisationEngine.h"

namespace ITMLib
{
	/* This helpful abstract interface allows you to ignore the fact that
	scenes are templates.
	*/
	class ITMMapGraphManager
	{
	public:
		virtual ~ITMMapGraphManager(void) {}

		virtual int createNewScene(void) = 0;
		virtual void removeScene(int index) = 0;
		virtual size_t numScenes(void) const = 0;

		virtual const ITMPoseConstraint & getRelation_const(int fromScene, int toScene) const = 0;
		virtual ITMPoseConstraint & getRelation(int fromScene, int toScene) = 0;
		virtual void eraseRelation(int fromScene, int toScene) = 0;
		virtual const ConstraintList & getConstraints(int sceneId) const = 0;

		virtual void setEstimatedGlobalPose(int sceneID, const ORUtils::SE3Pose & pose) = 0;
		virtual const ORUtils::SE3Pose & getEstimatedGlobalPose(int sceneID) const = 0;

		virtual bool resetTracking(int sceneID, const ORUtils::SE3Pose & pose) = 0;

		virtual const ORUtils::SE3Pose* getTrackingPose(int sceneID) const = 0;
		virtual int getSceneSize(int sceneID) const = 0;
		virtual int countVisibleBlocks(int sceneID, int minBlockId, int maxBlockId, bool invertIDs) const = 0;
	};

	template<class TVoxel, class TIndex>
	class ITMVoxelMapGraphManager : public ITMMapGraphManager
	{
	private:
		const ITMLibSettings *settings;
		const ITMVisualisationEngine<TVoxel, TIndex> *visualisationEngine;
		const ITMDenseMapper<TVoxel, TIndex> *denseMapper;
		Vector2i trackedImageSize;

		std::vector<ITMLocalMap<TVoxel, TIndex>*> allData;

	public:
		ITMVoxelMapGraphManager(const ITMLibSettings *settings, const ITMVisualisationEngine<TVoxel, TIndex> *visualisationEngine, const ITMDenseMapper<TVoxel, TIndex> *denseMapper, const Vector2i & trackedImageSize);
		~ITMVoxelMapGraphManager(void);

		int createNewScene(void);
		void removeScene(int index);
		size_t numScenes(void) const { return allData.size(); }

		const ITMLocalMap<TVoxel, TIndex>* getScene(int sceneID) const { return allData[sceneID]; }

		ITMLocalMap<TVoxel, TIndex>* getScene(int sceneID) { return allData[sceneID]; }

		const ITMPoseConstraint & getRelation_const(int fromScene, int toScene) const;
		ITMPoseConstraint & getRelation(int fromScene, int toScene);
		void eraseRelation(int fromScene, int toScene);
		const ConstraintList & getConstraints(int sceneId) const { return allData[sceneId]->relations; }

		void setEstimatedGlobalPose(int sceneID, const ORUtils::SE3Pose & pose) { allData[sceneID]->estimatedGlobalPose = pose; }
		const ORUtils::SE3Pose & getEstimatedGlobalPose(int sceneID) const { return allData[sceneID]->estimatedGlobalPose; }

		bool resetTracking(int sceneID, const ORUtils::SE3Pose & pose);
		const ORUtils::SE3Pose* getTrackingPose(int sceneID) const { return getScene(sceneID)->trackingState->pose_d; }

		int getSceneSize(int sceneID) const;
		int countVisibleBlocks(int sceneID, int minBlockId, int maxBlockId, bool invertIDs) const;

		ORUtils::SE3Pose findTransformation(int fromSceneID, int toSceneID) const;
	};
}