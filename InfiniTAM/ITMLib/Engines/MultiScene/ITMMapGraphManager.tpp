// Copyright 2016 Isis Innovation Limited and the authors of InfiniTAM

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
	int ITMVoxelMapGraphManager<TVoxel, TIndex>::createNewScene(void)
	{
		int newIdx = (int)allData.size();
		allData.push_back(new ITMLocalMap<TVoxel, TIndex>(settings, visualisationEngine, trackedImageSize));

		denseMapper->ResetScene(allData[newIdx]->scene);
		return newIdx;
	}

	template<class TVoxel, class TIndex>
	void ITMVoxelMapGraphManager<TVoxel, TIndex>::removeScene(int sceneID)
	{
		if ((sceneID < 0) || ((unsigned)sceneID >= allData.size())) return;

		// make sure there are no relations anywhere pointing to the scene
		const ConstraintList & l = getConstraints(sceneID);
		for (ConstraintList::const_iterator it = l.begin(); it != l.end(); ++it) eraseRelation(it->first, sceneID);

		// delete the scene
		delete allData[sceneID];
		allData.erase(allData.begin() + sceneID);
	}

	template<class TVoxel, class TIndex>
	ITMPoseConstraint & ITMVoxelMapGraphManager<TVoxel, TIndex>::getRelation(int fromScene, int toScene)
	{
		ConstraintList & m = getScene(fromScene)->relations;
		return m[toScene];
	}

	static const ITMPoseConstraint invalidPoseConstraint;

	template<class TVoxel, class TIndex>
	const ITMPoseConstraint & ITMVoxelMapGraphManager<TVoxel, TIndex>::getRelation_const(int fromScene, int toScene) const
	{
		if ((fromScene < 0) || (fromScene >= (int)allData.size())) return invalidPoseConstraint;

		const ConstraintList & m = getScene(fromScene)->relations;
		ConstraintList::const_iterator it = m.find(toScene);
		if (it == m.end()) return invalidPoseConstraint;

		return it->second;
	}

	template<class TVoxel, class TIndex>
	void ITMVoxelMapGraphManager<TVoxel, TIndex>::eraseRelation(int fromScene, int toScene)
	{
		if ((fromScene < 0) || (fromScene >= (int)allData.size())) return;

		std::map<int, ITMPoseConstraint> & m = getScene(fromScene)->relations;
		m.erase(toScene);
	}

	template<class TVoxel, class TIndex>
	bool ITMVoxelMapGraphManager<TVoxel, TIndex>::resetTracking(int sceneID, const ORUtils::SE3Pose & pose)
	{
		if ((sceneID < 0) || ((unsigned)sceneID >= allData.size())) return false;
		allData[sceneID]->trackingState->pose_d->SetFrom(&pose);
		allData[sceneID]->trackingState->age_pointCloud = -1;
		return true;
	}

	template<class TVoxel, class TIndex>
	int ITMVoxelMapGraphManager<TVoxel, TIndex>::getSceneSize(int sceneID) const
	{
		if ((sceneID < 0) || ((unsigned)sceneID >= allData.size())) return -1;

		ITMScene<TVoxel, TIndex> *scene = allData[sceneID]->scene;
		return scene->index.getNumAllocatedVoxelBlocks() - scene->localVBA.lastFreeBlockId - 1;
	}

	template<class TVoxel, class TIndex>
	int ITMVoxelMapGraphManager<TVoxel, TIndex>::countVisibleBlocks(int sceneID, int minBlockId, int maxBlockId, bool invertIDs) const
	{
		if ((sceneID < 0) || ((unsigned)sceneID >= allData.size())) return -1;
		const ITMLocalMap<TVoxel, TIndex> *scene = allData[sceneID];

		if (invertIDs) 
		{
			int tmp = minBlockId;
			minBlockId = scene->scene->index.getNumAllocatedVoxelBlocks() - maxBlockId - 1;
			maxBlockId = scene->scene->index.getNumAllocatedVoxelBlocks() - tmp - 1;
		}

		return visualisationEngine->CountVisibleBlocks(scene->scene, scene->renderState, minBlockId, maxBlockId);
	}

	struct LinkPathComparison 
	{
		bool operator()(const std::vector<int> & a, const std::vector<int> & b) { return a.size() > b.size(); }
	};

	template<class TVoxel, class TIndex>
	ORUtils::SE3Pose ITMVoxelMapGraphManager<TVoxel, TIndex>::findTransformation(int fromSceneID, int toSceneID) const
	{
		ORUtils::SE3Pose fromScenePose, toScenePose;
		if ((fromSceneID >= 0) || (fromSceneID < allData.size())) fromScenePose = allData[fromSceneID]->estimatedGlobalPose;
		if ((toSceneID >= 0) || (toSceneID < allData.size())) toScenePose = allData[toSceneID]->estimatedGlobalPose;
		return ORUtils::SE3Pose(toScenePose.GetM() * fromScenePose.GetInvM());
	}
}