// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMActiveSceneManager.h"

#include <queue>

using namespace ITMLib;

// try loop closures for this number of frames
static const int N_linktrials = 20;
// at least these many frames have to be tracked successfully
static const int N_linkoverlap = 10;
// try relocalisations for this number of frames
static const int N_reloctrials = 20;
// at least these many tracking attempts have to succeed for relocalisation
static const int N_relocsuccess = 10;
// threshold on number of blucks when a new local scene should be started
static const int N_maxblocknum = 10000;

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
void ITMLocalSceneManager_instance<TVoxel,TIndex>::addRelation(int fromScene, int toScene, const ITMPose & pose, int weight)
{
	if ((fromScene < 0)||((unsigned)fromScene >= allData.size())) return;
	if ((toScene < 0)||((unsigned)toScene >= allData.size())) return;

	ITMLocalScene<TVoxel,TIndex> *localScene = allData[fromScene];
	localScene->relations[toScene].AddObservation(pose, weight);

	ITMPose invPose(pose.GetInvM());
	localScene = allData[toScene];
	localScene->relations[fromScene].AddObservation(invPose, weight);
fprintf(stderr, "createSceneRelation %i -> %i\n", fromScene, toScene);
}

template<class TVoxel, class TIndex>
bool ITMLocalSceneManager_instance<TVoxel,TIndex>::resetTracking(int sceneID, const ITMPose & pose)
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
ITMPose ITMLocalSceneManager_instance<TVoxel,TIndex>::findTransformation(int fromSceneID, int toSceneID) const
{
	std::vector<int> pathThroughScenes = getShortestLinkPath(fromSceneID, toSceneID);
	ITMPose ret;

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

ITMActiveSceneManager::ITMActiveSceneManager(ITMLocalSceneManager *_localSceneManager)
{
	localSceneManager = _localSceneManager;
}

void ITMActiveSceneManager::initiateNewScene(bool isPrimaryScene)
{
	int newIdx = localSceneManager->createNewScene();

	ActiveDataDescriptor newLink;
	newLink.sceneIndex = newIdx;
	newLink.type = isPrimaryScene?PRIMARY_SCENE:NEW_SCENE;
	newLink.trackingAttempts = 0;
	activeData.push_back(newLink);
}

int ITMActiveSceneManager::initiateNewLink(int sceneID, const ITMPose & pose, bool isRelocalisation)
{
	static const bool ensureUniqueLinks = true;

	// make sure only one relocalisation per scene is attempted at a time
	if (ensureUniqueLinks) {
		for (size_t i = 0; i < activeData.size(); ++i) {
			if (activeData[i].sceneIndex == sceneID) return -1;
		}
	}

	if (!localSceneManager->resetTracking(sceneID, pose)) return -1;

	ActiveDataDescriptor newLink;
	newLink.sceneIndex = sceneID;
	newLink.type = isRelocalisation?RELOCALISATION:LOOP_CLOSURE;
	newLink.trackingAttempts = 0;
	activeData.push_back(newLink);
fprintf(stderr, "attempting relocalisation... %i (type %i)\n", sceneID, (int)isRelocalisation);
	return activeData.size()-1;
}

bool ITMActiveSceneManager::shouldStartNewArea(void) const
{
	int primarySceneIdx = -1;

	// don't start two new scenes at a time
	for (size_t i = 0; i < activeData.size(); ++i) {
		if (activeData[i].type == NEW_SCENE) return false;
		if (activeData[i].type == PRIMARY_SCENE) primarySceneIdx = activeData[i].sceneIndex;
	}

	if (primarySceneIdx < 0)
	{
		// TODO: check: if relocalisation fails for some time, start new scene
		return false;
	} else {
		int blocksInUse = localSceneManager->getSceneSize(primarySceneIdx);
		if (blocksInUse < N_maxblocknum) return false;

		return true;
	}

	return false;
}

bool ITMActiveSceneManager::shouldMovePrimaryScene(int newDataId, int bestDataId, int primaryDataId) const
{
	int sceneIdx_primary = -1;
	int sceneIdx_best = -1;
	int sceneIdx_new = -1;

	int blocksInUse_primary = -1;
	int blocksInUse_best = -1;
	bool isNewScene_best = false;
	int blocksInUse_new = -1;
	bool isNewScene_new = false;

	if (primaryDataId >= 0) sceneIdx_primary = activeData[primaryDataId].sceneIndex;
	if (bestDataId >= 0) sceneIdx_best = activeData[bestDataId].sceneIndex;
	if (newDataId >= 0) sceneIdx_new = activeData[newDataId].sceneIndex;

	// count blocks in all relevant scenes
	if (sceneIdx_primary >= 0) {
		blocksInUse_primary = localSceneManager->getSceneSize(sceneIdx_primary);
	}

	if (sceneIdx_new >= 0) {
		isNewScene_new = (activeData[newDataId].type == NEW_SCENE);
		blocksInUse_new = localSceneManager->getSceneSize(sceneIdx_new);
		if (blocksInUse_new < 0) return false;
	}

	if (sceneIdx_best >= 0) {
		isNewScene_best = (activeData[bestDataId].type == NEW_SCENE);
		blocksInUse_best = localSceneManager->getSceneSize(sceneIdx_best);
	}

	if (blocksInUse_primary < 0) {
		// TODO: if relocalisation fails, a new scene gets started,
		//       and is eventually accepted, this case will get relevant
		return true;
	}

	// step 1: is "new" better than "primary" ?

	// don't continue a local scene that is already full
	if (blocksInUse_new >= N_maxblocknum) return false;

	if (blocksInUse_new >= blocksInUse_primary) return false;

	// step 2: is "new" better than "best"?
	if (blocksInUse_best < 0) return true;

	// if this is a new scene, but we previously found that we can loop
	// close, don't accept the new scene!
	if (isNewScene_new && !isNewScene_best) return false;
	// if this is a loop closure and we have not found any alternative
	// loop closure before, accept the new one!
	if (!isNewScene_new && isNewScene_best) return true;

	// if the two are equal, take the smaller one
	return (blocksInUse_new < blocksInUse_best);
}

int ITMActiveSceneManager::findPrimaryDataIdx(void) const
{
	for (size_t i = 0; i < activeData.size(); ++i) {
		if (activeData[i].type == PRIMARY_SCENE) {
			return i;
		}
	}
	return -1;
}

int ITMActiveSceneManager::findPrimarySceneIdx(void) const
{
	int id = findPrimaryDataIdx();
	if (id < 0) return -1;
	return activeData[id].sceneIndex;
}

void ITMActiveSceneManager::recordTrackingResult(int dataID, int trackingSuccess, bool primaryTrackingSuccess)
{
	ActiveDataDescriptor & data = activeData[dataID];

	int primarySceneID = findPrimarySceneIdx();
	int sceneID = data.sceneIndex;
	data.trackingAttempts++;

	if (trackingSuccess >= 2)
	{
		if (data.type == RELOCALISATION)
		{
			data.constraints.push_back(localSceneManager->getTrackingPose(dataID)->GetM());
		}
		else if (((data.type == NEW_SCENE)||
		          (data.type == LOOP_CLOSURE))&&
		         primaryTrackingSuccess)
		{
			Matrix4f Tnew_inv = localSceneManager->getTrackingPose(sceneID)->GetInvM();
			Matrix4f Told = localSceneManager->getTrackingPose(primarySceneID)->GetM();
			Matrix4f Told_to_new = Tnew_inv * Told;

			data.constraints.push_back(Told_to_new);
		}
	}
	else if (trackingSuccess < 1)
	{
		if (data.type == PRIMARY_SCENE)
		{
			for (size_t j = 0; j < activeData.size(); ++j)
			{
				if (activeData[j].type == NEW_SCENE) activeData[j].type = LOST_NEW;
				else activeData[j].type = LOST;
			}
		}
	}
}

static float huber_weight(float residual, float b)
{
	double r_abs = fabs(residual);
	if (r_abs<b) return 1.0f;
	float tmp = 2.0f * b * r_abs - b*b;
	return sqrt(tmp)/r_abs;
}

static ITMPose estimateRelativePose(const std::vector<Matrix4f> & observations, int *out_numInliers, ITMPose *out_inlierPose)
{
	static const float huber_b = 0.1f;
	static const float weightsConverged = 0.01f;
	static const int maxIter = 10;
	std::vector<float> weights(observations.size(), 1.0f);
	std::vector<ITMPose> poses;

	for (size_t i = 0; i < observations.size(); ++i) {
		poses.push_back(ITMPose(observations[i]));
	}

	float params[6];
	for (int iter = 0; iter < maxIter; ++iter) {
		// estimate with fixed weights
		float sumweight = 0.0f;
		for (int j = 0; j < 6; ++j) params[j] = 0.0f;
		for (size_t i = 0; i < poses.size(); ++i) {
			for (int j = 0; j < 6; ++j) params[j] += weights[i] * poses[i].GetParams()[j];
			sumweight += weights[i];
		}
		for (int j = 0; j < 6; ++j) params[j] /= sumweight;

		// compute new weights
		float weightchanges = 0.0f;
		for (size_t i = 0; i < poses.size(); ++i) {
			float residual = 0.0f;
			for (int j = 0; j < 6; ++j) {
				float r = poses[i].GetParams()[j] - params[j];
				residual += r*r;
			}
			residual = sqrt(residual);
			float newweight = huber_weight(residual, huber_b);
			weightchanges += fabs(newweight - weights[i]);
			weights[i] = newweight;
		}
		float avgweightchange = weightchanges/poses.size();
		if (avgweightchange < weightsConverged) break;
	}

	int inliers = 0;
	Matrix4f inlierTrafo;
	inlierTrafo.setZeros();

	for (size_t i = 0; i < weights.size(); ++i) if (weights[i] > 0.8) {
		inlierTrafo += observations[i];
		++inliers;
	}
	if (out_inlierPose) out_inlierPose->SetM(inlierTrafo / (float)inliers);
	if (out_numInliers) *out_numInliers = inliers;

	return ITMPose(params);
}

int ITMActiveSceneManager::CheckSuccess_relocalisation(int dataID) const
{
	if (activeData[dataID].constraints.size() >= N_relocsuccess)
	{
		// sucessfully relocalised
		return 1;
	}
	if ((N_reloctrials - activeData[dataID].trackingAttempts) < (N_relocsuccess - (int)activeData[dataID].constraints.size()))
	{
		// relocalisation failed: declare as LOST
		return -1;
	}
	// keep trying
	return 0;
}

int ITMActiveSceneManager::CheckSuccess_newlink(int dataID, int *inliers, ITMPose *inlierPose) const
{
	const ActiveDataDescriptor & link = activeData[dataID];

	int inliers_local;
	ITMPose inlierPose_local;
	if (inliers == NULL) inliers = &inliers_local;
	if (inlierPose == NULL) inlierPose = &inlierPose_local;
	// TODO: take previous data from scene relations into account!

	estimateRelativePose(link.constraints, inliers, inlierPose);

fprintf(stderr, "trying to establish link ... -> %i: %i/%i attempts, %i/%i inliers\n", /*primaryDataIdx,*/ link.sceneIndex, link.trackingAttempts, N_linktrials, *inliers, N_linkoverlap);
	if (*inliers >= N_linkoverlap) {
		// accept link
		return 1;
	}
	if ((N_linktrials - link.trackingAttempts) < (N_linkoverlap - *inliers)) {
		// reject link
		return -1;
	}
	// keep trying
	return 0;
}

void ITMActiveSceneManager::AcceptNewLink(int fromData, int toData, const ITMPose & pose, int weight)
{
	int fromSceneIdx = activeData[fromData].sceneIndex;
	int toSceneIdx = activeData[toData].sceneIndex;

	localSceneManager->addRelation(fromSceneIdx, toSceneIdx, pose, weight);
}

void ITMActiveSceneManager::maintainActiveData(void)
{
	int primaryDataIdx = findPrimaryDataIdx();
	int moveToDataIdx = -1;
	for (size_t i = 0; i < activeData.size(); ++i)
	{
		ActiveDataDescriptor & link = activeData[i];

		if (link.type == RELOCALISATION)
		{
			int success = CheckSuccess_relocalisation(i);
			if (success == 1)
			{
				if (moveToDataIdx < 0) moveToDataIdx = i;
				else link.type = LOST;
fprintf(stderr, "relocalisation success, move to data %i\n", moveToDataIdx);
			}
			else if (success == -1)
			{
				link.type = LOST;
			}
		}
		if ((link.type == LOOP_CLOSURE)||
		    (link.type == NEW_SCENE))
		{
			ITMPose inlierPose; int inliers;
			int success = CheckSuccess_newlink(i, &inliers, &inlierPose);
			if (success == 1)
			{
				AcceptNewLink(primaryDataIdx, i, inlierPose, inliers);
				link.constraints.clear();
				link.trackingAttempts = 0;
				if (shouldMovePrimaryScene(i, moveToDataIdx, primaryDataIdx)) moveToDataIdx = i;
			}
			else if (success == -1)
			{
				if (link.type == NEW_SCENE) link.type = LOST_NEW;
				else link.type = LOST;
			}
		}
	}

	primaryDataIdx = -1;
	for (size_t i = 0; i < activeData.size(); ++i)
	{
		ActiveDataDescriptor & link = activeData[i];

		if ((signed)i == moveToDataIdx) link.type = PRIMARY_SCENE;

		if ((link.type == PRIMARY_SCENE)&&(moveToDataIdx >= 0)&&((signed)i != moveToDataIdx)) link.type = LOST;
		if ((link.type == NEW_SCENE)&&(moveToDataIdx >= 0)) link.type = LOST_NEW;
		if ((link.type == LOOP_CLOSURE)&&(moveToDataIdx >= 0)) link.type = LOST;
		if ((link.type == RELOCALISATION)&&(moveToDataIdx >= 0)) link.type = LOST;

		if (link.type == PRIMARY_SCENE)
		{
			if (primaryDataIdx >= 0) fprintf(stderr, "OOOPS, two or more primary scenes...\n");
			primaryDataIdx = i;
		}
	}

	for (size_t i = 0; i < activeData.size(); )
	{
		ActiveDataDescriptor & link = activeData[i];
		if (link.type == LOST_NEW)
		{
			localSceneManager->removeScene(link.sceneIndex);
			link.type = LOST;
		}
		if (link.type == LOST)
		{
			activeData.erase(activeData.begin()+i);
		} else {
			i++;
		}
	}
}

template class ITMLib::ITMLocalSceneManager_instance<ITMVoxel, ITMVoxelIndex>;

