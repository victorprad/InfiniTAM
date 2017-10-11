// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

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

		virtual int createNewLocalMap(void) = 0;
		virtual void removeLocalMap(int index) = 0;
		virtual size_t numLocalMaps(void) const = 0;

		virtual const ITMPoseConstraint & getRelation_const(int fromLocalMap, int toLocalMap) const = 0;
		virtual ITMPoseConstraint & getRelation(int fromLocalMap, int toLocalMap) = 0;
		virtual void eraseRelation(int fromLocalMap, int toLocalMap) = 0;
		virtual const ConstraintList & getConstraints(int localMapId) const = 0;

		virtual void setEstimatedGlobalPose(int localMapId, const ORUtils::SE3Pose & pose) = 0;
		virtual const ORUtils::SE3Pose & getEstimatedGlobalPose(int localMapId) const = 0;

		virtual bool resetTracking(int localMapId, const ORUtils::SE3Pose & pose) = 0;

		virtual const ORUtils::SE3Pose* getTrackingPose(int localMapId) const = 0;
		virtual int getLocalMapSize(int localMapId) const = 0;
		virtual int countVisibleBlocks(int localMapId, int minBlockId, int maxBlockId, bool invertIDs) const = 0;
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

		int createNewLocalMap(void);
		void removeLocalMap(int index);
		size_t numLocalMaps(void) const { return allData.size(); }

		const ITMLocalMap<TVoxel, TIndex>* getLocalMap(int localMapId) const { return allData[localMapId]; }

		ITMLocalMap<TVoxel, TIndex>* getLocalMap(int localMapId) { return allData[localMapId]; }

		const ITMPoseConstraint & getRelation_const(int fromLocalMap, int toLocalMap) const;
		ITMPoseConstraint & getRelation(int fromLocalMap, int toLocalMap);
		void eraseRelation(int fromLocalMap, int toLocalMap);
		const ConstraintList & getConstraints(int localMapId) const { return allData[localMapId]->relations; }

		void setEstimatedGlobalPose(int localMapId, const ORUtils::SE3Pose & pose) { allData[localMapId]->estimatedGlobalPose = pose; }
		const ORUtils::SE3Pose & getEstimatedGlobalPose(int localMapId) const { return allData[localMapId]->estimatedGlobalPose; }

		bool resetTracking(int localMapId, const ORUtils::SE3Pose & pose);
		const ORUtils::SE3Pose* getTrackingPose(int localMapId) const { return getLocalMap(localMapId)->trackingState->pose_d; }

		int getLocalMapSize(int localMapId) const;
		int countVisibleBlocks(int localMapId, int minBlockId, int maxBlockId, bool invertIDs) const;

		ORUtils::SE3Pose findTransformation(int fromlocalMapId, int tolocalMapId) const;
	};
}