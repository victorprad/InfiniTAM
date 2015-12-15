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
// When checking "overlap with original scene", find how many of the first N
// blocks are still visible
static const int N_originalblocks = 2000;
static const float F_originalBlocksThreshold = 0.1f;

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
	//fprintf(stderr, "createSceneRelation %i -> %i\n", fromScene, toScene);
}

template<class TVoxel, class TIndex>
void ITMLocalSceneManager_instance<TVoxel,TIndex>::getRelation(int fromScene, int toScene, ITMPose *out_pose, int *out_weight) const
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
	if (out_pose) *out_pose = ITMPose();
	if (out_weight) *out_weight = 0;
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

template<class TVoxel, class TIndex>
int ITMLocalSceneManager_instance<TVoxel,TIndex>::countVisibleBlocks(int sceneID, int minBlockId, int maxBlockId, bool invertIDs) const
{
	if ((sceneID < 0)||((unsigned)sceneID >= allData.size())) return -1.0f;
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
	//fprintf(stderr, "attempting relocalisation... %i (type %i)\n", sceneID, (int)isRelocalisation);
	return activeData.size()-1;
}

float ITMActiveSceneManager::visibleOriginalBlocks(int dataID) const
{
	int sceneID = activeData[dataID].sceneIndex;

	int allocated = localSceneManager->getSceneSize(sceneID);
	int counted = localSceneManager->countVisibleBlocks(sceneID, 0, N_originalblocks, true);
	//fprintf(stderr, "data %i: %i/%i (%i allocated)\n", dataID, counted, N_originalblocks, allocated);
	int tmp = N_originalblocks;
	if (allocated < tmp) tmp = allocated;
	return (float)counted / (float)tmp;
}

bool ITMActiveSceneManager::shouldStartNewArea(void) const
{
	int primarySceneIdx = -1;
	int primaryDataIdx = -1;

	// don't start two new scenes at a time
	for (size_t i = 0; i < activeData.size(); ++i) {
		if (activeData[i].type == NEW_SCENE) return false;
		if (activeData[i].type == PRIMARY_SCENE) {
			primaryDataIdx = i;
			primarySceneIdx = activeData[i].sceneIndex;
		}
	}

	if (primarySceneIdx < 0)
	{
		// TODO: check: if relocalisation fails for some time, start new scene
		return false;
	} else {
//		int blocksInUse = localSceneManager->getSceneSize(primarySceneIdx);
//		if (blocksInUse < N_maxblocknum) return false;
		float visibleRatio = visibleOriginalBlocks(primaryDataIdx);
		//fprintf(stderr, "original %f threshold %f\n", visibleRatio, F_originalBlocksThreshold);
		return visibleRatio < F_originalBlocksThreshold;

//		return true;
	}

	return false;
}

bool ITMActiveSceneManager::shouldMovePrimaryScene(int newDataId, int bestDataId, int primaryDataId) const
{
	int sceneIdx_primary = -1;
	int sceneIdx_best = -1;
	int sceneIdx_new = -1;

	int blocksInUse_primary = -1;
	float visibleRatio_primary = 1.0f;
	int blocksInUse_best = -1;
	float visibleRatio_best = 1.0f;
	bool isNewScene_best = false;
	int blocksInUse_new = -1;
	float visibleRatio_new = 1.0f;
	bool isNewScene_new = false;

	if (primaryDataId >= 0) sceneIdx_primary = activeData[primaryDataId].sceneIndex;
	if (bestDataId >= 0) sceneIdx_best = activeData[bestDataId].sceneIndex;
	if (newDataId >= 0) sceneIdx_new = activeData[newDataId].sceneIndex;

	// count blocks in all relevant scenes
	if (sceneIdx_primary >= 0) {
		blocksInUse_primary = localSceneManager->getSceneSize(sceneIdx_primary);
		visibleRatio_primary = visibleOriginalBlocks(primaryDataId);
	}

	if (sceneIdx_new >= 0) {
		isNewScene_new = (activeData[newDataId].type == NEW_SCENE);
		blocksInUse_new = localSceneManager->getSceneSize(sceneIdx_new);
		if (blocksInUse_new < 0) return false;
		visibleRatio_new = visibleOriginalBlocks(newDataId);
	}

	if (sceneIdx_best >= 0) {
		isNewScene_best = (activeData[bestDataId].type == NEW_SCENE);
		blocksInUse_best = localSceneManager->getSceneSize(sceneIdx_best);
		visibleRatio_best = visibleOriginalBlocks(bestDataId);
	}

	if (blocksInUse_primary < 0) {
		// TODO: if relocalisation fails, a new scene gets started,
		//       and is eventually accepted, this case will get relevant
		return true;
	}

	// step 1: is "new" better than "primary" ?

	// don't continue a local scene that is already full
/*	if (blocksInUse_new >= N_maxblocknum) return false;

	if (blocksInUse_new >= blocksInUse_primary) return false;*/
	if (visibleRatio_new <= visibleRatio_primary) return false;

	// step 2: is there any contender for a new scene to move to?
	if (blocksInUse_best < 0) return true;

	// if this is a new scene, but we previously found that we can loop
	// close, don't accept the new scene!
	if (isNewScene_new && !isNewScene_best) return false;
	// if this is a loop closure and we have not found any alternative
	// loop closure before, accept the new one!
	if (!isNewScene_new && isNewScene_best) return true;

	// if the two are equal, take the smaller one
	//return (blocksInUse_new < blocksInUse_best);
	return (visibleRatio_new > visibleRatio_best);
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

/** estimate a relative pose, taking into account a previous estimate (weight 0
    indicates that no previous estimate is available).
    out_numInliers and out_inlierPose is the number of inliers from amongst the
    new observations and the pose computed from them.
*/
static ITMPose estimateRelativePose(const std::vector<Matrix4f> & observations, const ITMPose & previousEstimate, float previousEstimate_weight, int *out_numInliers, ITMPose *out_inlierPose)
{
	static const float huber_b = 0.1f;
	static const float weightsConverged = 0.01f;
	static const int maxIter = 10;
	static const float inlierThresholdForFinalResult = 0.8f;
	std::vector<float> weights(observations.size()+1, 1.0f);
	std::vector<ITMPose> poses;

	for (size_t i = 0; i < observations.size(); ++i) {
		poses.push_back(ITMPose(observations[i]));
	}

	float params[6];
	for (int iter = 0; iter < maxIter; ++iter) {
		// estimate with fixed weights
		float sumweight = previousEstimate_weight;
		for (int j = 0; j < 6; ++j) params[j] = weights.back() * previousEstimate_weight * previousEstimate.GetParams()[j];
		for (size_t i = 0; i < poses.size(); ++i) {
			for (int j = 0; j < 6; ++j) params[j] += weights[i] * poses[i].GetParams()[j];
			sumweight += weights[i];
		}
		for (int j = 0; j < 6; ++j) params[j] /= sumweight;

		// compute new weights
		float weightchanges = 0.0f;
		for (size_t i = 0; i < weights.size(); ++i) {
			const ITMPose *p;
			float w = 1.0f;
			if (i < poses.size()) p = &(poses[i]);
			else {
				p = &(previousEstimate);
				w = previousEstimate_weight;
			}

			float residual = 0.0f;
			for (int j = 0; j < 6; ++j) {
				float r = p->GetParams()[j] - params[j];
				residual += r*r;
			}
			residual = sqrt(residual);
			float newweight = huber_weight(residual, huber_b);
			weightchanges += w * fabs(newweight - weights[i]);
			weights[i] = newweight;
		}
		float avgweightchange = weightchanges/(weights.size() - 1 + previousEstimate_weight);
		if (avgweightchange < weightsConverged) break;
	}

	int inliers = 0;
	Matrix4f inlierTrafo;
	inlierTrafo.setZeros();

	for (size_t i = 0; i < poses.size(); ++i) if (weights[i] > inlierThresholdForFinalResult) {
		inlierTrafo += observations[i];
		++inliers;
	}
	if (out_inlierPose) out_inlierPose->SetM(inlierTrafo / (float)MAX(inliers,1));
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

/*static void printPose(const ITMPose & p)
{
	fprintf(stderr, "%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", p.GetM().m00, p.GetM().m10, p.GetM().m20, p.GetM().m30, p.GetM().m01, p.GetM().m11, p.GetM().m21, p.GetM().m31, p.GetM().m02, p.GetM().m12, p.GetM().m22, p.GetM().m32);
}*/


int ITMActiveSceneManager::CheckSuccess_newlink(int dataID, int primaryDataID, int *inliers, ITMPose *inlierPose) const
{
	const ActiveDataDescriptor & link = activeData[dataID];

	// take previous data from scene relations into account!
	ITMPose previousEstimate;
	int previousEstimate_weight = 0;
	int primarySceneIndex = -1;
	if (primaryDataID >= 0) primarySceneIndex = activeData[primaryDataID].sceneIndex;
	localSceneManager->getRelation(primarySceneIndex, link.sceneIndex, &previousEstimate, &previousEstimate_weight);

	int inliers_local;
	ITMPose inlierPose_local;
	if (inliers == NULL) inliers = &inliers_local;
	if (inlierPose == NULL) inlierPose = &inlierPose_local;

	estimateRelativePose(link.constraints, previousEstimate, previousEstimate_weight, inliers, inlierPose);

	//fprintf(stderr, "trying to establish link %i -> %i: %i/%i attempts, %i/%i inliers\n", primarySceneIndex, link.sceneIndex, link.trackingAttempts, N_linktrials, *inliers, N_linkoverlap);
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
				//fprintf(stderr, "relocalisation success, move to data %i\n", moveToDataIdx);
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
			int success = CheckSuccess_newlink(i, primaryDataIdx, &inliers, &inlierPose);
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
			// NOTE: there will only be at most one new scene at
			// any given time and it's guaranteed to be the last
			// in the list. Removing this new scene will therefore
			// not require rearranging indices!
			localSceneManager->removeScene(link.sceneIndex);
			link.type = LOST;
		}
		if (link.type == LOST) activeData.erase(activeData.begin()+i);
		else i++;
	}

	// NOTE: this has to be done AFTER removing any previous new scene
	if (shouldStartNewArea()) initiateNewScene();
}

template class ITMLib::ITMLocalSceneManager_instance<ITMVoxel, ITMVoxelIndex>;

