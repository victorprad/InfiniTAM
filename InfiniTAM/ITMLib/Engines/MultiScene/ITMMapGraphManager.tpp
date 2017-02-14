// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMMapGraphManager.h"

//#include <queue>

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	ITMVoxelMapGraphManager<TVoxel, TIndex>::ITMVoxelMapGraphManager(const ITMLibSettings *_settings, const ITMVisualisationEngine<TVoxel, TIndex> *_visualisationEngine, const ITMDenseMapper<TVoxel, TIndex> *_denseMapper, const Vector2i & _trackedImageSize)
		: settings(_settings), visualisationEngine(_visualisationEngine), denseMapper(_denseMapper), trackedImageSize(_trackedImageSize)
	{
	}

	template<class TVoxel, class TIndex>
	ITMVoxelMapGraphManager<TVoxel, TIndex>::~ITMVoxelMapGraphManager(void)
	{
		while (allData.size() > 0)
		{
			delete allData.back();
			allData.pop_back();
		}
	}

	template<class TVoxel, class TIndex>
	int ITMVoxelMapGraphManager<TVoxel, TIndex>::createNewLocalMap(void)
	{
		int newIdx = (int)allData.size();
		allData.push_back(new ITMLocalMap<TVoxel, TIndex>(settings, visualisationEngine, trackedImageSize));

		denseMapper->ResetScene(allData[newIdx]->scene);
		return newIdx;
	}

	template<class TVoxel, class TIndex>
	void ITMVoxelMapGraphManager<TVoxel, TIndex>::removeLocalMap(int localMapId)
	{
		if ((localMapId < 0) || ((unsigned)localMapId >= allData.size())) return;

		// make sure there are no relations anywhere pointing to the local map
		const ConstraintList & l = getConstraints(localMapId);
		for (ConstraintList::const_iterator it = l.begin(); it != l.end(); ++it) eraseRelation(it->first, localMapId);

		// delete the local map
		delete allData[localMapId];
		allData.erase(allData.begin() + localMapId);
	}

	template<class TVoxel, class TIndex>
	ITMPoseConstraint & ITMVoxelMapGraphManager<TVoxel, TIndex>::getRelation(int fromLocalMap, int toLocalMap)
	{
		ConstraintList & m = getLocalMap(fromLocalMap)->relations;
		return m[toLocalMap];
	}

	static const ITMPoseConstraint invalidPoseConstraint;

	template<class TVoxel, class TIndex>
	const ITMPoseConstraint & ITMVoxelMapGraphManager<TVoxel, TIndex>::getRelation_const(int fromLocalMap, int toLocalMap) const
	{
		if ((fromLocalMap < 0) || (fromLocalMap >= (int)allData.size())) return invalidPoseConstraint;

		const ConstraintList & m = getLocalMap(fromLocalMap)->relations;
		ConstraintList::const_iterator it = m.find(toLocalMap);
		if (it == m.end()) return invalidPoseConstraint;

		return it->second;
	}

	template<class TVoxel, class TIndex>
	void ITMVoxelMapGraphManager<TVoxel, TIndex>::eraseRelation(int fromLocalMap, int toLocalMap)
	{
		if ((fromLocalMap < 0) || (fromLocalMap >= (int)allData.size())) return;

		std::map<int, ITMPoseConstraint> & m = getLocalMap(fromLocalMap)->relations;
		m.erase(toLocalMap);
	}

	template<class TVoxel, class TIndex>
	bool ITMVoxelMapGraphManager<TVoxel, TIndex>::resetTracking(int localMapId, const ORUtils::SE3Pose & pose)
	{
		if ((localMapId < 0) || ((unsigned)localMapId >= allData.size())) return false;
		allData[localMapId]->trackingState->pose_d->SetFrom(&pose);
		allData[localMapId]->trackingState->age_pointCloud = -1;
		return true;
	}

	template<class TVoxel, class TIndex>
	int ITMVoxelMapGraphManager<TVoxel, TIndex>::getLocalMapSize(int localMapId) const
	{
		if ((localMapId < 0) || ((unsigned)localMapId >= allData.size())) return -1;

		ITMScene<TVoxel, TIndex> *scene = allData[localMapId]->scene;
		return scene->index.getNumAllocatedVoxelBlocks() - scene->localVBA.lastFreeBlockId - 1;
	}

	template<class TVoxel, class TIndex>
	int ITMVoxelMapGraphManager<TVoxel, TIndex>::countVisibleBlocks(int localMapId, int minBlockId, int maxBlockId, bool invertIds) const
	{
		if ((localMapId < 0) || ((unsigned)localMapId >= allData.size())) return -1;
		const ITMLocalMap<TVoxel, TIndex> *localMap = allData[localMapId];

		if (invertIds) 
		{
			int tmp = minBlockId;
			minBlockId = localMap->scene->index.getNumAllocatedVoxelBlocks() - maxBlockId - 1;
			maxBlockId = localMap->scene->index.getNumAllocatedVoxelBlocks() - tmp - 1;
		}

		return visualisationEngine->CountVisibleBlocks(localMap->scene, localMap->renderState, minBlockId, maxBlockId);
	}

	struct LinkPathComparison 
	{
		bool operator()(const std::vector<int> & a, const std::vector<int> & b) { return a.size() > b.size(); }
	};

	template<class TVoxel, class TIndex>
	ORUtils::SE3Pose ITMVoxelMapGraphManager<TVoxel, TIndex>::findTransformation(int fromLocalMapId, int toLocalMapId) const
	{
		ORUtils::SE3Pose fromLocalMapPose, toLocalMapPose;
		if ((fromLocalMapId >= 0) || ((size_t)fromLocalMapId < allData.size())) fromLocalMapPose = allData[fromLocalMapId]->estimatedGlobalPose;
		if ((toLocalMapId >= 0) || ((size_t)toLocalMapId < allData.size())) toLocalMapPose = allData[toLocalMapId]->estimatedGlobalPose;
		return ORUtils::SE3Pose(toLocalMapPose.GetM() * fromLocalMapPose.GetInvM());
	}
}
