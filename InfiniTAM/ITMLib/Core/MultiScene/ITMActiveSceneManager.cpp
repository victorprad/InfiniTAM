// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMActiveSceneManager.h"

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

ITMActiveSceneManager::ITMActiveSceneManager(ITMMultiSceneManager *_localSceneManager)
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

int ITMActiveSceneManager::initiateNewLink(int sceneID, const ORUtils::SE3Pose & pose, bool isRelocalisation)
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
	return (int)activeData.size()-1;
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
	for (int i = 0; i < (int)activeData.size(); ++i) {
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
	for (int i = 0; i < (int)activeData.size(); ++i) {
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

int ITMActiveSceneManager::findBestVisualisationDataIdx(void) const
{
	int bestIdx = -1;
	for (int i = 0; i < static_cast<int>(activeData.size()); ++i) {
		if (activeData[i].type == PRIMARY_SCENE) return i;
		else if (activeData[i].type == NEW_SCENE) bestIdx = i;
		else if (activeData[i].type == RELOCALISATION) {
			if (bestIdx<0) { bestIdx = i; continue; }
			if (activeData[bestIdx].type == NEW_SCENE) continue;
			if (activeData[bestIdx].constraints.size() < activeData[i].constraints.size()) bestIdx = i;
		}
	}
	return bestIdx;
}

int ITMActiveSceneManager::findBestVisualisationSceneIdx(void) const
{
	int id = findBestVisualisationDataIdx();
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
	double tmp = 2.0 * b * r_abs - b*b;
	return (float)(sqrt(tmp)/r_abs);
}

/** estimate a relative pose, taking into account a previous estimate (weight 0
    indicates that no previous estimate is available).
    out_numInliers and out_inlierPose is the number of inliers from amongst the
    new observations and the pose computed from them.
*/
static ORUtils::SE3Pose estimateRelativePose(const std::vector<Matrix4f> & observations, const ORUtils::SE3Pose & previousEstimate, float previousEstimate_weight, int *out_numInliers, ORUtils::SE3Pose *out_inlierPose)
{
	static const float huber_b = 0.1f;
	static const float weightsConverged = 0.01f;
	static const int maxIter = 10;
	static const float inlierThresholdForFinalResult = 0.8f;
	std::vector<float> weights(observations.size()+1, 1.0f);
	std::vector<ORUtils::SE3Pose> poses;

	for (size_t i = 0; i < observations.size(); ++i) {
		poses.push_back(ORUtils::SE3Pose(observations[i]));
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
			const ORUtils::SE3Pose *p;
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

	return ORUtils::SE3Pose(params);
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

/*static void printPose(const ORUtils::SE3Pose & p)
{
	fprintf(stderr, "%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", p.GetM().m00, p.GetM().m10, p.GetM().m20, p.GetM().m30, p.GetM().m01, p.GetM().m11, p.GetM().m21, p.GetM().m31, p.GetM().m02, p.GetM().m12, p.GetM().m22, p.GetM().m32);
}*/


int ITMActiveSceneManager::CheckSuccess_newlink(int dataID, int primaryDataID, int *inliers, ORUtils::SE3Pose *inlierPose) const
{
	const ActiveDataDescriptor & link = activeData[dataID];

	// take previous data from scene relations into account!
	//ORUtils::SE3Pose previousEstimate;
	//int previousEstimate_weight = 0;
	int primarySceneIndex = -1;
	if (primaryDataID >= 0) primarySceneIndex = activeData[primaryDataID].sceneIndex;
	const ITMPoseConstraint & previousInformation = localSceneManager->getRelation(primarySceneIndex, link.sceneIndex);
	/* hmm... do we want the "Estimate" (i.e. the pose corrected by pose
	   graph optimization) or the "Observations" (i.e. the accumulated
	   poses seen in previous frames?
	   This should only really make a difference, if there is a large
	   disagreement between the two, in which case one might argue that
	   most likely something went wrong with a loop-closure, and we are
	   not really sure the "Estimate" is true or just based on an erroneous
	   loop closure. We therefore want to be consistent with previous
	   observations not estimations...
	*/
	//ORUtils::SE3Pose previousEstimate = previousInformation.GetEstimate();
	ORUtils::SE3Pose previousEstimate = previousInformation.GetAccumulatedObservations();
	int previousEstimate_weight = previousInformation.GetNumAccumulatedObservations();

	int inliers_local;
	ORUtils::SE3Pose inlierPose_local;
	if (inliers == NULL) inliers = &inliers_local;
	if (inlierPose == NULL) inlierPose = &inlierPose_local;

	estimateRelativePose(link.constraints, previousEstimate, (float)previousEstimate_weight, inliers, inlierPose);

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

void ITMActiveSceneManager::AcceptNewLink(int fromData, int toData, const ORUtils::SE3Pose & pose, int weight)
{
	int fromSceneIdx = activeData[fromData].sceneIndex;
	int toSceneIdx = activeData[toData].sceneIndex;

	//localSceneManager->addRelation(fromSceneIdx, toSceneIdx, pose, weight);
	ITMPoseConstraint & c = localSceneManager->getRelation(fromSceneIdx, toSceneIdx);
	c.AddObservation(pose, weight);
}

void ITMActiveSceneManager::maintainActiveData(void)
{
	int primaryDataIdx = findPrimaryDataIdx();
	int moveToDataIdx = -1;
	for (int i = 0; i < (int)activeData.size(); ++i)
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
			ORUtils::SE3Pose inlierPose; int inliers;
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
	for (int i = 0; i < (int)activeData.size(); ++i)
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
