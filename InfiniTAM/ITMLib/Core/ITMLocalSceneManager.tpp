// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMLocalSceneManager.h"

#include <queue>

namespace ITMLib
{

template<class TVoxel, class TIndex>
ITMLocalSceneManager_instance<TVoxel,TIndex>::ITMLocalSceneManager_instance(const ITMLibSettings *_settings, const ITMVisualisationEngine<TVoxel,TIndex> *_visualisationEngine, const ITMDenseMapper<TVoxel,TIndex> *_denseMapper, const Vector2i & _trackedImageSize)
	: settings(_settings), visualisationEngine(_visualisationEngine), denseMapper(_denseMapper), trackedImageSize(_trackedImageSize)
{ 
}

template<class TVoxel, class TIndex>
ITMLocalSceneManager_instance<TVoxel,TIndex>::~ITMLocalSceneManager_instance(void)
{
	while (allData.size()>0)
	{
		delete allData.back();
		allData.pop_back();
	}
}

template<class TVoxel, class TIndex>
int ITMLocalSceneManager_instance<TVoxel,TIndex>::createNewScene(void)
{
	int newIdx = (int)allData.size();
	allData.push_back(new ITMLocalScene<TVoxel,TIndex>(settings, visualisationEngine, trackedImageSize));

	denseMapper->ResetScene(allData[newIdx]->scene);
	return newIdx;
}

template<class TVoxel, class TIndex>
void ITMLocalSceneManager_instance<TVoxel,TIndex>::removeScene(int sceneID)
{
	if ((sceneID < 0)||((unsigned)sceneID >= allData.size())) return;
	delete allData[sceneID];
	allData.erase(allData.begin() + sceneID);
	// TODO: make sure there are no relations anywhere pointing to the scene just deleted
}

template<class TVoxel, class TIndex>
void ITMLocalSceneManager_instance<TVoxel,TIndex>::addRelation(int fromScene, int toScene, const ORUtils::SE3Pose & pose, int weight)
{
	if ((fromScene < 0)||((unsigned)fromScene >= allData.size())) return;
	if ((toScene < 0)||((unsigned)toScene >= allData.size())) return;

	ITMLocalScene<TVoxel,TIndex> *localScene = allData[fromScene];
	localScene->relations[toScene].AddObservation(pose, weight);

	ORUtils::SE3Pose invPose(pose.GetInvM());
	localScene = allData[toScene];
	localScene->relations[fromScene].AddObservation(invPose, weight);
	//fprintf(stderr, "createSceneRelation %i -> %i\n", fromScene, toScene);
}

template<class TVoxel, class TIndex>
void ITMLocalSceneManager_instance<TVoxel,TIndex>::getRelation(int fromScene, int toScene, ORUtils::SE3Pose *out_pose, int *out_weight) const
{
	if ((fromScene >= 0)&&((unsigned)fromScene < allData.size())) {
		const std::map<int,ITMPoseConstraint> & m = getScene(fromScene)->relations;
		std::map<int,ITMPoseConstraint>::const_iterator it = m.find(toScene);
		if (it != m.end()) {
			if (out_pose) *out_pose = it->second.GetAccumulatedInfo();
			if (out_weight) *out_weight = it->second.GetNumAccumulatedInfo();
			return;
		}
	}
	if (out_pose) *out_pose = ORUtils::SE3Pose();
	if (out_weight) *out_weight = 0;
}

template<class TVoxel, class TIndex>
bool ITMLocalSceneManager_instance<TVoxel,TIndex>::resetTracking(int sceneID, const ORUtils::SE3Pose & pose)
{
	if ((sceneID < 0)||((unsigned)sceneID >= allData.size())) return false;
	allData[sceneID]->trackingState->pose_d->SetFrom(&pose);
	allData[sceneID]->trackingState->age_pointCloud = -1;
	return true;
}

template<class TVoxel, class TIndex>
int ITMLocalSceneManager_instance<TVoxel,TIndex>::getSceneSize(int sceneID) const
{
	if ((sceneID < 0)||((unsigned)sceneID >= allData.size())) return -1;

	ITMScene<TVoxel,TIndex> *scene = allData[sceneID]->scene;
	return scene->index.getNumAllocatedVoxelBlocks() - scene->localVBA.lastFreeBlockId - 1;
}

template<class TVoxel, class TIndex>
int ITMLocalSceneManager_instance<TVoxel,TIndex>::countVisibleBlocks(int sceneID, int minBlockId, int maxBlockId, bool invertIDs) const
{
	if ((sceneID < 0)||((unsigned)sceneID >= allData.size())) return -1;
	const ITMLocalScene<TVoxel,TIndex> *scene = allData[sceneID];

	if (invertIDs) {
		int tmp = minBlockId;
		minBlockId = scene->scene->index.getNumAllocatedVoxelBlocks() - maxBlockId - 1;
		maxBlockId = scene->scene->index.getNumAllocatedVoxelBlocks() - tmp - 1;
	}

	return visualisationEngine->CountVisibleBlocks(scene->scene, scene->renderState, minBlockId, maxBlockId);
}

struct LinkPathComparison {
	public:
	bool operator()(const std::vector<int> & a, const std::vector<int> & b)
	{
		return a.size()>b.size();
	}
};

template<class TVoxel, class TIndex>
std::vector<int> ITMLocalSceneManager_instance<TVoxel,TIndex>::getShortestLinkPath(int fromSceneID, int toSceneID) const
{
	// this is an A*-search algorithm. it uses a priority queue, where the
	// least expensive path so far is always on top. let the STL do all
	// the sorting work...
	std::priority_queue<std::vector<int> > todoList;
	todoList.push(std::vector<int>(1, fromSceneID));

	while (todoList.size()>0) {
		std::vector<int> top = todoList.top();
		todoList.pop();

		int currentPos = top.back();
		if (currentPos == toSceneID) return top;

		for (std::map<int,ITMPoseConstraint>::const_iterator it = allData[currentPos]->relations.begin(); it != allData[currentPos]->relations.end(); ++it) {
			int newScene = it->first;
			bool cycle = false;
			for (size_t j = 0; j < top.size(); ++j) {
				if (top[j] == newScene) {
					cycle = true;
					break;
				}
			}
			if (cycle) continue;

			std::vector<int> newTop(top);
			newTop.push_back(newScene);
			todoList.push(newTop);
		}
	}

	// no route found
	return std::vector<int>();
}

template<class TVoxel, class TIndex>
ORUtils::SE3Pose ITMLocalSceneManager_instance<TVoxel,TIndex>::findTransformation(int fromSceneID, int toSceneID) const
{
	std::vector<int> pathThroughScenes = getShortestLinkPath(fromSceneID, toSceneID);
	ORUtils::SE3Pose ret;

	for (size_t i = 0; i+1 < pathThroughScenes.size(); ++i) {
		int currentScene = pathThroughScenes[i];
		int nextScene = pathThroughScenes[i+1];
		std::map<int,ITMPoseConstraint>::const_iterator it = allData[currentScene]->relations.find(nextScene);
		// it should never be invalid, as the path was just
		// constructed in that way
		ret.SetM(it->second.GetAccumulatedInfo().GetM() * ret.GetM());
		ret.Coerce();
	}

	return ret;
}

}
