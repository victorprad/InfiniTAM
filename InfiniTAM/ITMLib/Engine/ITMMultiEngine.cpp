// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMMultiEngine.h"

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
// number of nearest neighbours to find in the loop closure detection
static const int k_loopcloseneighbours = 5;
// maximum distance reported by LCD library to attempt relocalisation
static const float F_maxdistattemptreloc = 0.2f;

ITMMultiEngine::ITMMultiEngine(const ITMLibSettings *settings, const ITMRGBDCalib *calib, Vector2i imgSize_rgb, Vector2i imgSize_d)
{
	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

	this->settings = settings;

	switch (settings->deviceType)
	{
	case ITMLibSettings::DEVICE_CPU:
		lowLevelEngine = new ITMLowLevelEngine_CPU();
		viewBuilder = new ITMViewBuilder_CPU(calib);
		visualisationEngine = new ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>();
		break;
	case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
		lowLevelEngine = new ITMLowLevelEngine_CUDA();
		viewBuilder = new ITMViewBuilder_CUDA(calib);
		visualisationEngine = new ITMVisualisationEngine_CUDA<ITMVoxel, ITMVoxelIndex>();
#endif
		break;
	case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
		lowLevelEngine = new ITMLowLevelEngine_Metal();
		viewBuilder = new ITMViewBuilder_Metal(calib);
		visualisationEngine = new ITMVisualisationEngine_Metal<ITMVoxel, ITMVoxelIndex>();
#endif
		break;
	}

	renderState_freeview = NULL; //will be created by the visualisation engine

	denseMapper = new ITMDenseMapper<ITMVoxel, ITMVoxelIndex>(settings);

	imuCalibrator = new ITMIMUCalibrator_iPad();
	tracker = ITMTrackerFactory<ITMVoxel, ITMVoxelIndex>::Instance().Make(imgSize_rgb, imgSize_d, settings, lowLevelEngine, imuCalibrator, NULL/*scene TODO: this will fail for Ren Tracker*/);
	trackingController = new ITMTrackingController(tracker, visualisationEngine, settings);
	trackedImageSize = ITMTrackingController::GetTrackedImageSize(tracker, imgSize_rgb, imgSize_d);

//	primaryDataIdx = 0;
	freeviewDataIdx = 0;
	AddNewLocalScene(-1);

	tracker->UpdateInitialPose(allData[0]->trackingState);

	view = NULL; // will be allocated by the view builder

	mLoopClosureDetector = new LCDLib::LoopClosureDetector(imgSize_d, Vector2f(0.5f,3.0f), 0.2f, 500, 4);
}

ITMMultiEngine::~ITMMultiEngine()
{
	while (allData.size()>0)
	{
		delete allData.back();
		allData.pop_back();
	}

	if (renderState_freeview!=NULL) delete renderState_freeview;

	delete denseMapper;
	delete trackingController;

	delete tracker;
	delete imuCalibrator;

	delete lowLevelEngine;
	delete viewBuilder;

	if (view != NULL) delete view;

	delete visualisationEngine;

	delete mLoopClosureDetector;
}

void ITMMultiEngine::AddNewLocalScene(int primarySceneIdx)
{
	int newIdx = (int)allData.size();
	allData.push_back(new ITMLocalScene<ITMVoxel,ITMVoxelIndex>(settings, visualisationEngine, trackingController, trackedImageSize));

	denseMapper->ResetScene(allData.back()->scene);

//	if (primarySceneIdx<0) return;

	ActiveDataDescriptor newLink;
	newLink.sceneIndex = newIdx;
	newLink.type = (primarySceneIdx<0)?ActiveDataDescriptor::PRIMARY_SCENE:ActiveDataDescriptor::NEW_SCENE;
	newLink.trackingAttempts = 0;
	activeData.push_back(newLink);
}

bool ITMMultiEngine::AddNewRelocalisationScene(int sceneID, int primarySceneIdx, const ITMPose & pose)
{
	static const bool ensureUniqueLinks = true;
	bool trackingLost = (primarySceneIdx < 0);
	if (sceneID == primarySceneIdx) return false;
	if ((unsigned)sceneID >= allData.size()) return false;

	// make sure only one relocalisation per scene is attempted at a time
	if (ensureUniqueLinks) {
		for (size_t i = 0; i < activeData.size(); ++i) {
			if (activeData[i].sceneIndex == sceneID) return false;
		}
	}

	allData[sceneID]->trackingState->pose_d->SetFrom(&pose);
	allData[sceneID]->trackingState->age_pointCloud = -1;

	ActiveDataDescriptor newLink;
	newLink.sceneIndex = sceneID;
	newLink.type = trackingLost?ActiveDataDescriptor::RELOCALISATION:ActiveDataDescriptor::LOOP_CLOSURE;
	newLink.trackingAttempts = 0;
	activeData.push_back(newLink);
fprintf(stderr, "attempting relocalisation... %i %i (type %i)\n", sceneID, primarySceneIdx, (int)trackingLost);
	return true;
}

void ITMMultiEngine::DiscardLocalScene(int sceneID)
{
	if ((sceneID < 0)||((unsigned)sceneID >= allData.size())) return;
	delete allData[sceneID];
	allData.erase(allData.begin() + sceneID);
	// TODO: make sure there are no relations anywhere pointing to the scene just deleted
}

void ITMMultiEngine::ChangeFreeviewDataIdx(ITMPose *pose, int newIdx)
{
	if (allData[freeviewDataIdx]->relations.find(newIdx) == allData[freeviewDataIdx]->relations.end()) {
		// cant jump... :(
		return;
	} else {
		ITMPose trafo = allData[freeviewDataIdx]->relations[newIdx].GetAccumulatedInfo();
		pose->SetM(pose->GetM()*trafo.GetInvM());
		pose->Coerce();
		freeviewDataIdx = newIdx;
	}
}

bool ITMMultiEngine::shouldStartNewArea(void) const
{
	int primarySceneIdx = -1;

	// don't start two new scenes at a time
	for (size_t i = 0; i < activeData.size(); ++i) {
		if (activeData[i].type == ActiveDataDescriptor::NEW_SCENE) return false;
		if (activeData[i].type == ActiveDataDescriptor::PRIMARY_SCENE) primarySceneIdx = activeData[i].sceneIndex;
	}

	if (primarySceneIdx < 0)
	{
		// TODO: check: if relocalisation fails for some time, start new scene
		return false;
	} else {
		ITMScene<ITMVoxel,ITMVoxelIndex> *scene = allData[primarySceneIdx]->scene;
		int blocksInUse = scene->index.getNumAllocatedVoxelBlocks()-scene->localVBA.lastFreeBlockId - 1;
		if (blocksInUse < N_maxblocknum) return false;

		return true;
	}

	return false;
}

bool ITMMultiEngine::shouldMovePrimaryScene(int newDataIdx, int bestDataIdx, int primaryDataIdx) const
{
	// ensure valid parameters
	if ((newDataIdx < 0) || ((unsigned)newDataIdx >= allData.size())) return false;

	int blocksInUse_primary = -1;
	int blocksInUse_best = -1;
	bool isNewScene_best = false;
	int blocksInUse_new = -1;
	bool isNewScene_new = false;

	// count blocks in all relevant scenes
	ITMLocalScene<ITMVoxel,ITMVoxelIndex> *localScene;
	if (primaryDataIdx >= 0) {
		localScene = allData[activeData[primaryDataIdx].sceneIndex];
		blocksInUse_primary = localScene->scene->index.getNumAllocatedVoxelBlocks() - localScene->scene->localVBA.lastFreeBlockId - 1;
	}

	if (newDataIdx >= 0) {
		isNewScene_new = (activeData[newDataIdx].type == ActiveDataDescriptor::NEW_SCENE);
		localScene = allData[activeData[newDataIdx].sceneIndex];
		blocksInUse_new = localScene->scene->index.getNumAllocatedVoxelBlocks() - localScene->scene->localVBA.lastFreeBlockId - 1;
	}

	if (bestDataIdx >= 0) {
		isNewScene_best = (activeData[bestDataIdx].type == ActiveDataDescriptor::NEW_SCENE);
		localScene = allData[activeData[bestDataIdx].sceneIndex];
		blocksInUse_best = localScene->scene->index.getNumAllocatedVoxelBlocks() - localScene->scene->localVBA.lastFreeBlockId - 1;
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

int ITMMultiEngine::FindPrimaryDataIdx(void) const
{
	for (size_t i = 0; i < activeData.size(); ++i) {
		if (activeData[i].type == ActiveDataDescriptor::PRIMARY_SCENE) {
			return i;
		}
	}
	return -1;
}

ITMTrackingState* ITMMultiEngine::GetTrackingState(void)
{
	int idx = FindPrimaryDataIdx();
	if (idx < 0) idx = 0;
	return allData[idx]->trackingState;
}

static float huber_weight(float residual, float b)
{
	double r_abs = fabs(residual);
	if (r_abs<b) return 1.0f;
	float tmp = 2.0f * b * r_abs - b*b;
	return sqrt(tmp)/r_abs;
}

ITMPose ITMMultiEngine::EstimateRelativePose(const std::vector<Matrix4f> & observations, int *out_numInliers, ITMPose *out_inlierPose)
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
/*fprintf(stderr, "new weights:");
for (size_t i = 0; i < poses.size(); ++i) {fprintf(stderr, " %f", weights[i]);}
fprintf(stderr, "\n");*/
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

int ITMMultiEngine::CheckSuccess_relocalisation(int dataID) const
{
	if (activeData[dataID].constraints.size() >= N_relocsuccess)
	{
		// sucessfully relocalised
		return 1;
		//primaryTrackingSuccess = true;
		//primaryDataIdx = dataID;
	}
	if (activeData[dataID].trackingAttempts >= N_reloctrials)
	{
		return -1;
		// relocalisation failed: declare as LOST
		//activeData[dataID].type = ActiveDataDescriptor::LOST;
		//todoList[i].prepare = false;
	}
	return 0;
}

int ITMMultiEngine::CheckSuccess_newlink(int dataID, int *inliers, ITMPose *inlierPose) const
{
	const ActiveDataDescriptor & link = activeData[dataID];

	int inliers_local;
	ITMPose inlierPose_local;
	if (inliers == NULL) inliers = &inliers_local;
	if (inlierPose == NULL) inlierPose = &inlierPose_local;
	// TODO: take previous data from scene relations into account!

	EstimateRelativePose(link.constraints, inliers, inlierPose);

fprintf(stderr, "trying to establish link ... -> %i: %i/%i attempts, %i/%i inliers\n", /*primaryDataIdx,*/ link.sceneIndex, link.trackingAttempts, N_linktrials, *inliers, N_linkoverlap);
	if (*inliers >= N_linkoverlap) {
		// accept link
/*		if (shouldMovePrimaryScene(link.sceneIndex, moveToSceneIdx)) {
			moveToSceneIdx = link.sceneIndex;
		}*/
//		activeData[dataID].trackingAttempts = -inliers;
//		return true;
		return 1;
	}
	if (link.trackingAttempts > N_linktrials) {
		return -1;
//		if (isnewScene) activeData[dataID].type = ActiveDataDescriptor::LOST_NEW;
//		else activeData[dataID].type = ActiveDataDescriptor::LOST;
	}
	return 0;
}

void ITMMultiEngine::AcceptNewLink(int dataId, int primaryDataId, ITMPose pose, int weight)
{
	ActiveDataDescriptor & link = activeData[dataId];
	int primarySceneIdx = activeData[primaryDataId].sceneIndex;

	ITMLocalScene<ITMVoxel,ITMVoxelIndex> *localScene = allData[primarySceneIdx];
	localScene->relations[link.sceneIndex].AddObservation(pose, weight);

	ITMPose invPose(pose.GetInvM());
	localScene = allData[link.sceneIndex];
	localScene->relations[primarySceneIdx].AddObservation(invPose, weight);
}

/*
    - whenever a new local scene is added, add to list of "to be established 3D relations"
    - whenever a relocalisation is detected, add to the same list, preserving any existing information on that 3D relation

    - for all 3D relations to be established:
      - attempt tracking in both scenes
      - if success, add to list of new candidates
      - if less than n_overlap "new candidates" in more than n_reloctrialframes frames, discard
      - if at least n_overlap "new candidates":
        - try to compute 3D relation, weighting old information accordingly
        - if outlier ratio below p_relation_outliers and at least n_overlap inliers, success
*/
struct TodoListEntry {
	TodoListEntry(int _activeDataID, bool _track, bool _fusion, bool _prepare, bool _updateLink)
	  : dataID(_activeDataID), track(_track), fusion(_fusion), prepare(_prepare), updateLink(_updateLink), preprepare(false) {}
	TodoListEntry(void) {}
	int dataID;
	bool track;
	bool fusion;
	bool prepare;
	bool updateLink;
	bool preprepare;
};

void ITMMultiEngine::ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement)
{
	// prepare image and turn it into a depth image
	if (imuMeasurement==NULL) viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter);
	else viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, imuMeasurement);

	std::vector<TodoListEntry> todoList;

	// find primary data, if available
	int primaryDataIdx = FindPrimaryDataIdx();
// TODO: move elsewhere! it might as well stay here, but might be nicer elsewhere
	if ((primaryDataIdx>=0)&&(shouldStartNewArea())) {
		AddNewLocalScene(activeData[primaryDataIdx].sceneIndex);
	}

	// if there is a "primary data index", process it
	if (primaryDataIdx >= 0) {
		todoList.push_back(TodoListEntry(primaryDataIdx, true, true, true, false));
	}
	// also make sure to process all "relocalisations"
	// usually this implies "no primary data index"
	for (size_t i = 0; i < activeData.size(); ++i) {
		if (activeData[i].type == ActiveDataDescriptor::RELOCALISATION) {
			todoList.push_back(TodoListEntry(i, true, false, true, false));
		}
	}
	// make sure the list is not empty from the start!
	if (todoList.size() == 0) {
		todoList.push_back(TodoListEntry(-1, false, false, false, false));
	}

	// tracking
	fprintf(stderr, "tracking data blocks:");
	bool primaryTrackingSuccess = false;
	bool detectedSimilarKeyframes = false;
	for (size_t i = 0; i < todoList.size(); ++i)
	{
		ITMLocalScene<ITMVoxel,ITMVoxelIndex> *currentScene = NULL;
		if (todoList[i].dataID >= 0) {
			currentScene = allData[activeData[todoList[i].dataID].sceneIndex];
		}

		if (todoList[i].preprepare) {
			// this is typically happening once to initiate relocalisation/loop closure
			denseMapper->UpdateVisibleList(view, currentScene->trackingState, currentScene->scene, currentScene->renderState);
			trackingController->Prepare(currentScene->trackingState, currentScene->scene, view, currentScene->renderState);
		}

		if (todoList[i].track)
		{
			int dataID = todoList[i].dataID;
			int blocksInUse = currentScene->scene->index.getNumAllocatedVoxelBlocks()-currentScene->scene->localVBA.lastFreeBlockId - 1;
			fprintf(stderr, " %i%s (%i)", /*activeData[dataID].sceneIndex*/activeData[dataID].sceneIndex, (todoList[i].dataID==primaryDataIdx)?"*":"", blocksInUse);

			ITMPose oldPose(*(currentScene->trackingState->pose_d));
			trackingController->Track(currentScene->trackingState, view);
			activeData[dataID].trackingAttempts++;

			int trackingSuccess = 0;
			if (currentScene->trackingState->poseQuality>0.8f) trackingSuccess = 2;
			else if (currentScene->trackingState->poseQuality>0.4f) trackingSuccess = 1;

			if (trackingSuccess < 2)
			{
				todoList[i].fusion = false;
				todoList[i].updateLink = false;
			}
			if (trackingSuccess < 1)
			{
				todoList[i].prepare = false;
				*(currentScene->trackingState->pose_d) = oldPose;
			}

// TODO: mark some more scenes as "LOST" if they are lost... possibly...

			if (activeData[dataID].type == ActiveDataDescriptor::RELOCALISATION)
			{
				if (trackingSuccess >= 2)
				{
					activeData[dataID].constraints.push_back(currentScene->trackingState->pose_d->GetM());
				}
			}
			else if (activeData[dataID].type == ActiveDataDescriptor::PRIMARY_SCENE)
			{
//fprintf(stderr, "primary scene tracking status: %i\n", trackingSuccess);
				if (trackingSuccess >= 2) primaryTrackingSuccess = true;
				// just lost track of the primary scene?
				else if (trackingSuccess < 1)
				{
					for (size_t j = 0; j < activeData.size(); ++j)
					{
						if (activeData[j].type == ActiveDataDescriptor::NEW_SCENE) activeData[j].type = ActiveDataDescriptor::LOST_NEW;
						else activeData[j].type = ActiveDataDescriptor::LOST;
					}
					todoList.resize(i+1);
					primaryDataIdx = -1;
fprintf(stderr, "Lost track of primary scene\n");
				}
			}
		}

		// - first tracking pass is for primary scene or ongoing
		//   relocalisation attempts
		// - second tracking pass will be about (newly detected) loop
		//   closures, relocalisations, etc.
		// If we are at the end of the ToDo list, try loop closure
		// detection or relocalisation, if we've not done that before.
		if ((i == (todoList.size() - 1))&&(!detectedSimilarKeyframes)) {
			// first refresh the todolist
			for (size_t i = 0; i < activeData.size(); ++i) {
				if (activeData[i].type == ActiveDataDescriptor::NEW_SCENE) todoList.push_back(TodoListEntry(i, true, true, true, true));
				else if (activeData[i].type == ActiveDataDescriptor::LOOP_CLOSURE) todoList.push_back(TodoListEntry(i, true, false, true, true));
				//else if (activeData[i].type == ActiveDataDescriptor::RELOCALISATION) todoList.push_back(TodoListEntry(i, true, false, true, false));
			}

fprintf(stderr, "LCD\n");
			int NN[k_loopcloseneighbours];
			float distances[k_loopcloseneighbours];
			int addKeyframeIdx = mLoopClosureDetector->ProcessFrame(view->depth, k_loopcloseneighbours, NN, distances, primaryTrackingSuccess);
			detectedSimilarKeyframes = true;
			int primarySceneIdx = -1;
			if (primaryDataIdx >= 0) primarySceneIdx = activeData[primaryDataIdx].sceneIndex;

			// add keyframe, if necessary
			if (addKeyframeIdx >= 0)
			{
				poseDatabase.storePose(addKeyframeIdx, *(allData[primarySceneIdx]->trackingState->pose_d), primarySceneIdx);
			}
			else for (int j = 0; j < k_loopcloseneighbours; ++j)
			{
				if (distances[j] > F_maxdistattemptreloc) continue;
				const LCDLib::PoseDatabase::PoseInScene & keyframe = poseDatabase.retrievePose(NN[j]);
				if (AddNewRelocalisationScene(keyframe.sceneIdx, primarySceneIdx, keyframe.pose) && (primarySceneIdx<0)) {
					// this is a new relocalisation attempt
					TodoListEntry todoItem(activeData.size()-1, true, false, true, false);
					todoItem.preprepare = true;
					todoList.push_back(todoItem);
				}
			}
		}

		// fusion
		if (todoList[i].fusion) {
			denseMapper->ProcessFrame(view, currentScene->trackingState, currentScene->scene, currentScene->renderState);
		} else if (todoList[i].prepare) {
			denseMapper->UpdateVisibleList(view, currentScene->trackingState, currentScene->scene, currentScene->renderState);
		}

		// raycast to renderState_live for tracking and free visualisation
		if (todoList[i].prepare) {
			trackingController->Prepare(currentScene->trackingState, currentScene->scene, view, currentScene->renderState);
		}

		// update all links and 3D relations
		if ((primaryTrackingSuccess)&&(todoList[i].updateLink)) {
			Matrix4f Tnew_inv = currentScene->trackingState->pose_d->GetInvM();
			Matrix4f Told = allData[activeData[primaryDataIdx].sceneIndex]->trackingState->pose_d->GetM();
			Matrix4f Told_to_new = Tnew_inv * Told;

			activeData[todoList[i].dataID].constraints.push_back(Told_to_new);
		}
	}

	int moveToDataIdx = -1;
	for (size_t i = 0; i < activeData.size(); ++i)
	{
		ActiveDataDescriptor & link = activeData[i];

		if (link.type == ActiveDataDescriptor::RELOCALISATION)
		{
			int success = CheckSuccess_relocalisation(i);
			if (success == 1)
			{
				if (moveToDataIdx < 0) moveToDataIdx = i;
				else link.type = ActiveDataDescriptor::LOST;
fprintf(stderr, "relocalisation success, move to data %i\n", moveToDataIdx);
			}
			else if (success == -1)
			{
				link.type = ActiveDataDescriptor::LOST;
			}
		}
		if ((link.type == ActiveDataDescriptor::LOOP_CLOSURE)||
		    (link.type == ActiveDataDescriptor::NEW_SCENE))
		{
			ITMPose inlierPose; int inliers;
			int success = CheckSuccess_newlink(i, &inliers, &inlierPose);
			if (success == 1)
			{
				AcceptNewLink(i, primaryDataIdx, inlierPose, inliers);
				link.constraints.clear();
				link.trackingAttempts = 0;
				if (shouldMovePrimaryScene(i, moveToDataIdx, primaryDataIdx)) {
					moveToDataIdx = i;
				}
			}
			if (success == -1)
			{
				if (link.type == ActiveDataDescriptor::NEW_SCENE) link.type = ActiveDataDescriptor::LOST_NEW;
				else link.type = ActiveDataDescriptor::LOST;
			}
		}
	}

	primaryDataIdx = -1;
	for (size_t i = 0; i < activeData.size(); ++i)
	{
		ActiveDataDescriptor & link = activeData[i];

		if ((signed)i == moveToDataIdx) {
			link.type = ActiveDataDescriptor::PRIMARY_SCENE;
		}

		if ((link.type == ActiveDataDescriptor::PRIMARY_SCENE)&&(moveToDataIdx >= 0)&&((signed)i != moveToDataIdx)) link.type = ActiveDataDescriptor::LOST;
		if ((link.type == ActiveDataDescriptor::NEW_SCENE)&&(moveToDataIdx >= 0)) link.type = ActiveDataDescriptor::LOST_NEW;
		if ((link.type == ActiveDataDescriptor::LOOP_CLOSURE)&&(moveToDataIdx >= 0)) link.type = ActiveDataDescriptor::LOST;
		if ((link.type == ActiveDataDescriptor::RELOCALISATION)&&(moveToDataIdx >= 0)) link.type = ActiveDataDescriptor::LOST;

		if (link.type == ActiveDataDescriptor::PRIMARY_SCENE) {
			if (primaryDataIdx >= 0) fprintf(stderr, "OOOPS, two or more primary scenes...\n");
			primaryDataIdx = i;
		}
	}

	for (size_t i = 0; i < activeData.size(); )
	{
		ActiveDataDescriptor & link = activeData[i];
		if (link.type == ActiveDataDescriptor::LOST_NEW)
		{
			DiscardLocalScene(link.sceneIndex);
			link.type = ActiveDataDescriptor::LOST;
		}
		if (link.type == ActiveDataDescriptor::LOST)
		{
			activeData.erase(activeData.begin()+i);
		} else {
			i++;
		}
	}

fprintf(stderr, "...done!\n");

#if 0
	// compute relative pose information
	for (int j = 0; (unsigned)j < activeDataIdx.size(); ++j) {
		int jIdx = activeDataIdx[j];
		Matrix4f Tj_inv = allData[jIdx]->trackingState->pose_d->GetInvM();
		for (int i = 0; (unsigned)i < activeDataIdx.size(); ++i) {
			int iIdx = activeDataIdx[i];
			if (iIdx == jIdx) continue;

			Matrix4f Ti = allData[iIdx]->trackingState->pose_d->GetM();
			Matrix4f Tji = Tj_inv * Ti;

			allRelations[iIdx][jIdx].AddObservation(Tji);
		}
	}
	for (int i = 0; (unsigned)i < activeDataIdx.size(); ++i) {
		int iIdx = activeDataIdx[i];
		for (int j = 0; (unsigned)j < activeDataIdx.size(); ++j) {
			int jIdx = activeDataIdx[j];
			if (iIdx == jIdx) continue;

			fprintf(stderr, "relative pose %i -> %i\n", iIdx,jIdx);
			Matrix4f Tji = allRelations[iIdx][jIdx].GetAccumulatedInfo();
			Matrix4f Tij;
			Tji.inv(Tij);
			for (int r = 0; r < 4; ++r) {
			fprintf(stderr, "%f %f %f %f    %f %f %f %f\n",
				Tji.at(0,r), Tji.at(1,r), Tji.at(2,r), Tji.at(3,r),
				Tij.at(0,r), Tij.at(1,r), Tij.at(2,r), Tij.at(3,r));
			}
		}
	}
#endif
}

Vector2i ITMMultiEngine::GetImageSize(void) const
{
	return trackedImageSize;
}

void ITMMultiEngine::GetImage(ITMUChar4Image *out, GetImageType getImageType, ITMPose *pose, ITMIntrinsics *intrinsics)
{
	if (view == NULL) return;

	out->Clear();

	switch (getImageType)
	{
	case ITMMultiEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
		out->ChangeDims(view->rgb->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) 
			out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	case ITMMultiEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
		out->ChangeDims(view->depth->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depth->UpdateHostFromDevice();
		ITMVisualisationEngine<ITMVoxel,ITMVoxelIndex>::DepthToUchar4(out, view->depth);
		break;
	case ITMMultiEngine::InfiniTAM_IMAGE_SCENERAYCAST:
	{
		int primaryDataIdx = FindPrimaryDataIdx();
		if (primaryDataIdx < 0) break; // TODO: clear image? what else to do when tracking is lost?
		ITMLocalScene<ITMVoxel,ITMVoxelIndex> *activeScene = allData[activeData[primaryDataIdx].sceneIndex];
		ORUtils::Image<Vector4u> *srcImage = activeScene->renderState->raycastImage;
		out->ChangeDims(srcImage->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
			out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);	
		break;
	}
	case ITMMultiEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED:
	case ITMMultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
	case ITMMultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
	{
		IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;
		ITMLocalScene<ITMVoxel,ITMVoxelIndex> *activeData = allData[freeviewDataIdx];
		if (getImageType == ITMMultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME;
		else if (getImageType == ITMMultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
		if (renderState_freeview == NULL) renderState_freeview = visualisationEngine->CreateRenderState(activeData->scene, out->noDims);

		visualisationEngine->FindVisibleBlocks(activeData->scene, pose, intrinsics, renderState_freeview);
		visualisationEngine->CreateExpectedDepths(activeData->scene, pose, intrinsics, renderState_freeview);
		visualisationEngine->RenderImage(activeData->scene, pose, intrinsics, renderState_freeview, renderState_freeview->raycastImage, type);

		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
			out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	}
	case ITMMultiEngine::InfiniTAM_IMAGE_UNKNOWN:
		break;
	};
}

