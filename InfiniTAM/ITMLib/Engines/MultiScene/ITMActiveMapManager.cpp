// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMActiveMapManager.h"

using namespace ITMLib;

// try loop closures for this number of frames
static const int N_linktrials = 20;
// at least these many frames have to be tracked successfully
static const int N_linkoverlap = 10;
// try relocalisations for this number of frames
static const int N_reloctrials = 20;
// at least these many tracking attempts have to succeed for relocalisation
static const int N_relocsuccess = 10;
// When checking "overlap with original local map", find how many of the first N
// blocks are still visible
static const int N_originalblocks = 1000;
static const float F_originalBlocksThreshold = 0.2f; //0.4f

ITMActiveMapManager::ITMActiveMapManager(ITMMapGraphManager *_localMapManager)
{
	localMapManager = _localMapManager;
}

int ITMActiveMapManager::initiateNewLocalMap(bool isPrimaryLocalMap)
{
	int newIdx = localMapManager->createNewLocalMap();

	ActiveDataDescriptor newLink;
	newLink.localMapIndex = newIdx;
	newLink.type = isPrimaryLocalMap ? PRIMARY_LOCAL_MAP : NEW_LOCAL_MAP;
	newLink.trackingAttempts = 0;
	activeData.push_back(newLink);

	return newIdx;
}

int ITMActiveMapManager::initiateNewLink(int localMapId, const ORUtils::SE3Pose & pose, bool isRelocalisation)
{
	static const bool ensureUniqueLinks = true;

	// make sure only one relocalisation per local map is attempted at a time
	if (ensureUniqueLinks) {
		for (size_t i = 0; i < activeData.size(); ++i) {
			if (activeData[i].localMapIndex == localMapId) return -1;
		}
	}

	if (!localMapManager->resetTracking(localMapId, pose)) return -1;

	ActiveDataDescriptor newLink;
	newLink.localMapIndex = localMapId;
	newLink.type = isRelocalisation ? RELOCALISATION : LOOP_CLOSURE;
	newLink.trackingAttempts = 0;
	activeData.push_back(newLink);
	
	return (int)activeData.size() - 1;
}

float ITMActiveMapManager::visibleOriginalBlocks(int dataID) const
{
	int localMapId = activeData[dataID].localMapIndex;

	int allocated = localMapManager->getLocalMapSize(localMapId);
	int counted = localMapManager->countVisibleBlocks(localMapId, 0, N_originalblocks, true);
	
	int tmp = N_originalblocks;
	if (allocated < tmp) tmp = allocated;
	return (float)counted / (float)tmp;
}

bool ITMActiveMapManager::shouldStartNewArea(void) const
{
	int primaryLocalMapIdx = -1;
	int primaryDataIdx = -1;

	// don't start two new local maps at a time
	for (int i = 0; i < (int)activeData.size(); ++i)
	{
		if (activeData[i].type == NEW_LOCAL_MAP) return false;
		if (activeData[i].type == PRIMARY_LOCAL_MAP) 
		{
			primaryDataIdx = i;
			primaryLocalMapIdx = activeData[i].localMapIndex;
		}
	}

	// TODO: check: if relocalisation fails for some time, start new local map
	if (primaryLocalMapIdx < 0) return false;
	else return visibleOriginalBlocks(primaryDataIdx) < F_originalBlocksThreshold;

	return false;
}

bool ITMActiveMapManager::shouldMovePrimaryLocalMap(int newDataId, int bestDataId, int primaryDataId) const
{
	int localMapIdx_primary = -1;
	int localMapIdx_best = -1;
	int localMapIdx_new = -1;

	int blocksInUse_primary = -1;
	float visibleRatio_primary = 1.0f;
	int blocksInUse_best = -1;
	float visibleRatio_best = 1.0f;
	bool isNewLocalMap_best = false;
	int blocksInUse_new = -1;
	float visibleRatio_new = 1.0f;
	bool isNewLocalMap_new = false;

	if (primaryDataId >= 0) localMapIdx_primary = activeData[primaryDataId].localMapIndex;
	if (bestDataId >= 0) localMapIdx_best = activeData[bestDataId].localMapIndex;
	if (newDataId >= 0) localMapIdx_new = activeData[newDataId].localMapIndex;

	// count blocks in all relevant localMaps
	if (localMapIdx_primary >= 0) 
	{
		blocksInUse_primary = localMapManager->getLocalMapSize(localMapIdx_primary);
		visibleRatio_primary = visibleOriginalBlocks(primaryDataId);
	}

	if (localMapIdx_new >= 0) 
	{
		isNewLocalMap_new = (activeData[newDataId].type == NEW_LOCAL_MAP);
		blocksInUse_new = localMapManager->getLocalMapSize(localMapIdx_new);
		if (blocksInUse_new < 0) return false;
		visibleRatio_new = visibleOriginalBlocks(newDataId);
	}

	if (localMapIdx_best >= 0) 
	{
		isNewLocalMap_best = (activeData[bestDataId].type == NEW_LOCAL_MAP);
		blocksInUse_best = localMapManager->getLocalMapSize(localMapIdx_best);
		visibleRatio_best = visibleOriginalBlocks(bestDataId);
	}

	if (blocksInUse_primary < 0) {
		// TODO: if relocalisation fails, a new local map gets started,
		//       and is eventually accepted, this case will get relevant
		return true;
	}

	// step 1: is "new" better than "primary" ?

	// don't continue a local map that is already full
/*	if (blocksInUse_new >= N_maxblocknum) return false;

	if (blocksInUse_new >= blocksInUse_primary) return false;*/
	if (visibleRatio_new <= visibleRatio_primary) return false;

	// step 2: is there any contender for a new local map to move to?
	if (blocksInUse_best < 0) return true;

	// if this is a new local map, but we previously found that we can loop
	// close, don't accept the new local map!
	if (isNewLocalMap_new && !isNewLocalMap_best) return false;
	// if this is a loop closure and we have not found any alternative
	// loop closure before, accept the new one!
	if (!isNewLocalMap_new && isNewLocalMap_best) return true;

	// if the two are equal, take the smaller one
	//return (blocksInUse_new < blocksInUse_best);
	return (visibleRatio_new > visibleRatio_best);
}

int ITMActiveMapManager::findPrimaryDataIdx(void) const
{
	for (int i = 0; i < (int)activeData.size(); ++i) 
		if (activeData[i].type == PRIMARY_LOCAL_MAP) return i;

	return -1;
}

int ITMActiveMapManager::findPrimaryLocalMapIdx(void) const
{
	int id = findPrimaryDataIdx();
	if (id < 0) return -1;
	return activeData[id].localMapIndex;
}

int ITMActiveMapManager::findBestVisualisationDataIdx(void) const
{
	int bestIdx = -1;
	for (int i = 0; i < static_cast<int>(activeData.size()); ++i) 
	{
		if (activeData[i].type == PRIMARY_LOCAL_MAP) return i;
		else if (activeData[i].type == NEW_LOCAL_MAP) bestIdx = i;
		else if (activeData[i].type == RELOCALISATION) 
		{
			if (bestIdx < 0) { bestIdx = i; continue; }
			if (activeData[bestIdx].type == NEW_LOCAL_MAP) continue;
			if (activeData[bestIdx].constraints.size() < activeData[i].constraints.size()) bestIdx = i;
		}
	}
	return bestIdx;
}

int ITMActiveMapManager::findBestVisualisationLocalMapIdx(void) const
{
	int id = findBestVisualisationDataIdx();
	if (id < 0) return -1;
	return activeData[id].localMapIndex;
}

void ITMActiveMapManager::recordTrackingResult(int dataID, ITMTrackingState::TrackingResult trackingResult, bool primaryTrackingSuccess)
{
	ActiveDataDescriptor & data = activeData[dataID];

	int primaryLocalMapID = findPrimaryLocalMapIdx();
	int localMapId = data.localMapIndex;
	data.trackingAttempts++;

	if (trackingResult == ITMTrackingState::TRACKING_GOOD)
	{
		if (data.type == RELOCALISATION) data.constraints.push_back(localMapManager->getTrackingPose(dataID)->GetM());
		else if (((data.type == NEW_LOCAL_MAP) || (data.type == LOOP_CLOSURE)) && primaryTrackingSuccess)
		{
			Matrix4f Tnew_inv = localMapManager->getTrackingPose(localMapId)->GetInvM();
			Matrix4f Told = localMapManager->getTrackingPose(primaryLocalMapID)->GetM();
			Matrix4f Told_to_new = Tnew_inv * Told;

			data.constraints.push_back(Told_to_new);
		}
	}
	else if (trackingResult == ITMTrackingState::TRACKING_FAILED)
	{
		if (data.type == PRIMARY_LOCAL_MAP)
		{
			for (size_t j = 0; j < activeData.size(); ++j)
			{
				if (activeData[j].type == NEW_LOCAL_MAP) activeData[j].type = LOST_NEW;
				else activeData[j].type = LOST;
			}
		}
	}
}

static float huber_weight(float residual, float b)
{
	double r_abs = fabs(residual);
	if (r_abs < b) return 1.0f;
	double tmp = 2.0 * b * r_abs - b*b;
	return (float)(sqrt(tmp) / r_abs);
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
	std::vector<float> weights(observations.size() + 1, 1.0f);
	std::vector<ORUtils::SE3Pose> poses;

	for (size_t i = 0; i < observations.size(); ++i) poses.push_back(ORUtils::SE3Pose(observations[i]));

	float params[6];
	for (int iter = 0; iter < maxIter; ++iter) 
	{
		// estimate with fixed weights
		float sumweight = previousEstimate_weight;
		for (int j = 0; j < 6; ++j) params[j] = weights.back() * previousEstimate_weight * previousEstimate.GetParams()[j];
		for (size_t i = 0; i < poses.size(); ++i) 
		{
			for (int j = 0; j < 6; ++j) params[j] += weights[i] * poses[i].GetParams()[j];
			sumweight += weights[i];
		}
		for (int j = 0; j < 6; ++j) params[j] /= sumweight;

		// compute new weights
		float weightchanges = 0.0f;
		for (size_t i = 0; i < weights.size(); ++i) 
		{
			const ORUtils::SE3Pose *p;
			float w = 1.0f;
			if (i < poses.size()) p = &(poses[i]);
			else
			{
				p = &(previousEstimate);
				w = previousEstimate_weight;
			}

			float residual = 0.0f;
			for (int j = 0; j < 6; ++j) 
			{
				float r = p->GetParams()[j] - params[j];
				residual += r*r;
			}
			residual = sqrt(residual);
			float newweight = huber_weight(residual, huber_b);
			weightchanges += w * fabs(newweight - weights[i]);
			weights[i] = newweight;
		}

		float avgweightchange = weightchanges / (weights.size() - 1 + previousEstimate_weight);
		if (avgweightchange < weightsConverged) break;
	}

	int inliers = 0;
	Matrix4f inlierTrafo;
	inlierTrafo.setZeros();

	for (size_t i = 0; i < poses.size(); ++i) if (weights[i] > inlierThresholdForFinalResult) 
	{
		inlierTrafo += observations[i];
		++inliers;
	}
	if (out_inlierPose) out_inlierPose->SetM(inlierTrafo / (float)MAX(inliers, 1));
	if (out_numInliers) *out_numInliers = inliers;

	return ORUtils::SE3Pose(params);
}

int ITMActiveMapManager::CheckSuccess_relocalisation(int dataID) const
{
	// sucessfully relocalised
	if (activeData[dataID].constraints.size() >= N_relocsuccess) return 1;

	// relocalisation failed: declare as LOST
	if ((N_reloctrials - activeData[dataID].trackingAttempts) < (N_relocsuccess - (int)activeData[dataID].constraints.size())) return -1;

	// keep trying
	return 0;
}

int ITMActiveMapManager::CheckSuccess_newlink(int dataID, int primaryDataID, int *inliers, ORUtils::SE3Pose *inlierPose) const
{
	const ActiveDataDescriptor & link = activeData[dataID];

	// take previous data from local map relations into account!
	//ORUtils::SE3Pose previousEstimate;
	//int previousEstimate_weight = 0;
	int primaryLocalMapIndex = -1;
	if (primaryDataID >= 0) primaryLocalMapIndex = activeData[primaryDataID].localMapIndex;
	const ITMPoseConstraint & previousInformation = localMapManager->getRelation_const(primaryLocalMapIndex, link.localMapIndex);
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

	ORUtils::SE3Pose previousEstimate = previousInformation.GetAccumulatedObservations();
	int previousEstimate_weight = previousInformation.GetNumAccumulatedObservations();

	int inliers_local;
	ORUtils::SE3Pose inlierPose_local;
	if (inliers == NULL) inliers = &inliers_local;
	if (inlierPose == NULL) inlierPose = &inlierPose_local;

	estimateRelativePose(link.constraints, previousEstimate, (float)previousEstimate_weight, inliers, inlierPose);

	// accept link
	if (*inliers >= N_linkoverlap) return 1;

	// reject link
	if ((N_linktrials - link.trackingAttempts) < (N_linkoverlap - *inliers)) return -1;

	// keep trying
	return 0;
}

void ITMActiveMapManager::AcceptNewLink(int fromData, int toData, const ORUtils::SE3Pose & pose, int weight)
{
	int fromLocalMapIdx = activeData[fromData].localMapIndex;
	int toLocalMapIdx = activeData[toData].localMapIndex;

	{
		ITMPoseConstraint &c = localMapManager->getRelation(fromLocalMapIdx, toLocalMapIdx);
		c.AddObservation(pose, weight);
	}
	{
		ORUtils::SE3Pose invPose(pose.GetInvM());
		ITMPoseConstraint &c = localMapManager->getRelation(toLocalMapIdx, fromLocalMapIdx);
		c.AddObservation(invPose, weight);
	}
}

bool ITMActiveMapManager::maintainActiveData(void)
{
	bool localMapGraphChanged = false;

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
			}
			else if (success == -1) link.type = LOST;
		}

		if ((link.type == LOOP_CLOSURE) || (link.type == NEW_LOCAL_MAP))
		{
			ORUtils::SE3Pose inlierPose; int inliers;

			int success = CheckSuccess_newlink(i, primaryDataIdx, &inliers, &inlierPose);
			if (success == 1)
			{
				AcceptNewLink(primaryDataIdx, i, inlierPose, inliers);
				link.constraints.clear();
				link.trackingAttempts = 0;
				if (shouldMovePrimaryLocalMap(i, moveToDataIdx, primaryDataIdx)) moveToDataIdx = i;
				localMapGraphChanged = true;
			}
			else if (success == -1)
			{
				if (link.type == NEW_LOCAL_MAP) link.type = LOST_NEW;
				else link.type = LOST;
			}
		}
	}

	std::vector<int> restartLinksToLocalMaps;
	primaryDataIdx = -1;
	for (int i = 0; i < (int)activeData.size(); ++i)
	{
		ActiveDataDescriptor & link = activeData[i];

		if ((signed)i == moveToDataIdx) link.type = PRIMARY_LOCAL_MAP;

		if ((link.type == PRIMARY_LOCAL_MAP) && (moveToDataIdx >= 0) && ((signed)i != moveToDataIdx)) 
		{
			link.type = LOST;
			restartLinksToLocalMaps.push_back(link.localMapIndex);
		}
		
		if ((link.type == NEW_LOCAL_MAP) && (moveToDataIdx >= 0)) link.type = LOST_NEW;
		
		if ((link.type == LOOP_CLOSURE) && (moveToDataIdx >= 0)) 
		{
			link.type = LOST;
			restartLinksToLocalMaps.push_back(link.localMapIndex);
		}

		if ((link.type == RELOCALISATION) && (moveToDataIdx >= 0)) link.type = LOST;

		if (link.type == PRIMARY_LOCAL_MAP)
		{
			if (primaryDataIdx >= 0) fprintf(stderr, "OOOPS, two or more primary localMaps...\n");
			primaryDataIdx = i;
		}
	}

	for (size_t i = 0; i < activeData.size(); )
	{
		ActiveDataDescriptor & link = activeData[i];
		if (link.type == LOST_NEW)
		{
			// NOTE: there will only be at most one new local map at
			// any given time and it's guaranteed to be the last
			// in the list. Removing this new local map will therefore
			// not require rearranging indices!
			localMapManager->removeLocalMap(link.localMapIndex);
			link.type = LOST;
		}
		if (link.type == LOST) activeData.erase(activeData.begin() + i);
		else i++;
	}

	for (std::vector<int>::const_iterator it = restartLinksToLocalMaps.begin(); it != restartLinksToLocalMaps.end(); ++it) {
		initiateNewLink(*it, *(localMapManager->getTrackingPose(*it)), false);
	}

	// NOTE: this has to be done AFTER removing any previous new local map
	if (shouldStartNewArea())
	{
		int newIdx = initiateNewLocalMap();

		if (primaryDataIdx >= 0)
		{
			int primaryLocalMapIdx = activeData[primaryDataIdx].localMapIndex;
			localMapManager->setEstimatedGlobalPose(newIdx, ORUtils::SE3Pose(localMapManager->getTrackingPose(primaryLocalMapIdx)->GetM() * localMapManager->getEstimatedGlobalPose(primaryLocalMapIdx).GetM()));
		}
	}

	return localMapGraphChanged;
}
