// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMMultiEngine.h"

using namespace ITMLib;

#if 0
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
#endif
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
	trackingController = new ITMTrackingController(tracker, settings);
	trackedImageSize = trackingController->GetTrackedImageSize(imgSize_rgb, imgSize_d);

	freeviewSceneIdx = 0;
	sceneManager = new ITMLocalSceneManager_instance<ITMVoxel,ITMVoxelIndex>(settings, visualisationEngine, denseMapper, trackedImageSize);
	activeDataManager = new ITMActiveSceneManager(sceneManager);
	activeDataManager->initiateNewScene(true);

//TODO	tracker->UpdateInitialPose(allData[0]->trackingState);

	view = NULL; // will be allocated by the view builder

	mLoopClosureDetector = new LCDLib::LoopClosureDetector(imgSize_d, Vector2f(0.5f,3.0f), 0.2f, 500, 4);
}

ITMMultiEngine::~ITMMultiEngine(void)
{
	delete activeDataManager;
	delete sceneManager;

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

void ITMMultiEngine::changeFreeviewSceneIdx(ITMPose *pose, int newIdx)
{
//	if ((newIdx < 0)||((unsigned)newIdx >= sceneManager->numScenes())) return;
	if (newIdx < 0) newIdx = (int)sceneManager->numScenes()-1;
	if ((unsigned)newIdx >= sceneManager->numScenes()) newIdx = 0;

	ITMPose trafo = sceneManager->findTransformation(freeviewSceneIdx, newIdx);
	pose->SetM(pose->GetM() * trafo.GetInvM());
	pose->Coerce();
	freeviewSceneIdx = newIdx;
}

ITMTrackingState* ITMMultiEngine::GetTrackingState(void)
{
	int idx = activeDataManager->findPrimarySceneIdx();
	if (idx < 0) idx = 0;
	return sceneManager->getScene(idx)->trackingState;
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
	TodoListEntry(int _activeDataID, bool _track, bool _fusion, bool _prepare)
	  : dataID(_activeDataID), track(_track), fusion(_fusion), prepare(_prepare), preprepare(false) {}
	TodoListEntry(void) {}
	int dataID;
	bool track;
	bool fusion;
	bool prepare;
	bool preprepare;
};

void ITMMultiEngine::ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement)
{
	// prepare image and turn it into a depth image
	if (imuMeasurement==NULL) viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter);
	else viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, imuMeasurement);

	std::vector<TodoListEntry> todoList;

	// find primary data, if available
	int primaryDataIdx = activeDataManager->findPrimaryDataIdx();

	// if there is a "primary data index", process it
	if (primaryDataIdx >= 0) {
		todoList.push_back(TodoListEntry(primaryDataIdx, true, true, true));
	}
	// after primary scene, make sure to process all relocalisations, new
	// scenes and loop closures
	for (int i = 0; i < activeDataManager->numActiveScenes(); ++i) {
		if (activeDataManager->getSceneType(i) == ITMActiveSceneManager::NEW_SCENE) todoList.push_back(TodoListEntry(i, true, true, true));
		else if (activeDataManager->getSceneType(i) == ITMActiveSceneManager::LOOP_CLOSURE) todoList.push_back(TodoListEntry(i, true, false, true));
		else if (activeDataManager->getSceneType(i) == ITMActiveSceneManager::RELOCALISATION) todoList.push_back(TodoListEntry(i, true, false, true));
	}

	// finally, once all is done, call the loop closure detection engine
	todoList.push_back(TodoListEntry(-1, false, false, false));

	fprintf(stderr, "tracking data blocks:");
	bool primaryTrackingSuccess = false;
	for (size_t i = 0; i < todoList.size(); ++i)
	{
		// - first pass of the todo list is for primary scene and
		//   ongoing relocalisation and loopclosure attempts
		// - an element with id -1 marks the end of the first pass,
		//   a request to call the loop closure detection engine, and
		//   the start of the second pass
		// - second tracking pass will be about newly detected loop
		//   closures, relocalisations, etc.
		if (todoList[i].dataID == -1) {
			fprintf(stderr, " LCD");
			int NN[k_loopcloseneighbours];
			float distances[k_loopcloseneighbours];
			int addKeyframeIdx = mLoopClosureDetector->ProcessFrame(view->depth, k_loopcloseneighbours, NN, distances, primaryTrackingSuccess);
			int primarySceneIdx = -1;
			if (primaryDataIdx >= 0) primarySceneIdx = activeDataManager->getSceneIndex(primaryDataIdx);

			// add keyframe, if necessary
			if (addKeyframeIdx >= 0)
			{
				poseDatabase.storePose(addKeyframeIdx, *(sceneManager->getScene(primarySceneIdx)->trackingState->pose_d), primarySceneIdx);
			}
			else for (int j = 0; j < k_loopcloseneighbours; ++j)
			{
				if (distances[j] > F_maxdistattemptreloc) continue;
				const LCDLib::PoseDatabase::PoseInScene & keyframe = poseDatabase.retrievePose(NN[j]);
				int newDataIdx = activeDataManager->initiateNewLink(keyframe.sceneIdx, keyframe.pose, (primarySceneIdx<0));
				if (newDataIdx >= 0) {
					// this is a new relocalisation attempt
					TodoListEntry todoItem(newDataIdx, true, false, true);
					todoItem.preprepare = true;
					todoList.push_back(todoItem);
				}
			}
			continue;
		}

		ITMLocalScene<ITMVoxel,ITMVoxelIndex> *currentScene = NULL;
		int currentSceneIdx = activeDataManager->getSceneIndex(todoList[i].dataID);
		currentScene = sceneManager->getScene(currentSceneIdx);

		// if a new relocalisation/loopclosure is started, this will
		// do the initial raycasting before tracking can start
		if (todoList[i].preprepare) {
			denseMapper->UpdateVisibleList(view, currentScene->trackingState, currentScene->scene, currentScene->renderState);
			trackingController->Prepare(currentScene->trackingState, currentScene->scene, view, visualisationEngine, currentScene->renderState);
		}

		// tracking
		if (todoList[i].track)
		{
			int dataID = todoList[i].dataID;
			int blocksInUse = currentScene->scene->index.getNumAllocatedVoxelBlocks()-currentScene->scene->localVBA.lastFreeBlockId - 1;
			fprintf(stderr, " %i%s (%i)", /*activeData[dataID].sceneIndex*/currentSceneIdx, (todoList[i].dataID==primaryDataIdx)?"*":"", blocksInUse);

			ITMPose oldPose(*(currentScene->trackingState->pose_d));
			trackingController->Track(currentScene->trackingState, view);

			int trackingSuccess = 0;
			if (currentScene->trackingState->poseQuality>0.8f) trackingSuccess = 2;
			else if (currentScene->trackingState->poseQuality>0.4f) trackingSuccess = 1;

			if (trackingSuccess < 2)
			{
				todoList[i].fusion = false;
			}
			if (trackingSuccess < 1)
			{
				todoList[i].prepare = false;
				*(currentScene->trackingState->pose_d) = oldPose;
			}

			// TODO: mark some more scenes as "LOST" if they are lost... possibly...

			if (activeDataManager->getSceneType(dataID) == ITMActiveSceneManager::PRIMARY_SCENE)
			{
				if (trackingSuccess >= 2) primaryTrackingSuccess = true;
				else if (trackingSuccess < 1)
				{
					primaryDataIdx = -1;
					todoList.resize(i+1);
					todoList.push_back(TodoListEntry(-1, false, false, false));
fprintf(stderr, "Lost track of primary scene\n");
				}
			}
			activeDataManager->recordTrackingResult(dataID, trackingSuccess, primaryTrackingSuccess);
		}

		// fusion
		if (todoList[i].fusion) {
			denseMapper->ProcessFrame(view, currentScene->trackingState, currentScene->scene, currentScene->renderState);
		} else if (todoList[i].prepare) {
			denseMapper->UpdateVisibleList(view, currentScene->trackingState, currentScene->scene, currentScene->renderState);
		}

		// raycast to renderState_live for tracking and free visualisation
		if (todoList[i].prepare) {
			trackingController->Prepare(currentScene->trackingState, currentScene->scene, view, visualisationEngine, currentScene->renderState);
		}
	}

	activeDataManager->maintainActiveData();

	fprintf(stderr, "...done!\n");
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
		int primarySceneIdx = activeDataManager->findPrimarySceneIdx();
		if (primarySceneIdx < 0) break; // TODO: clear image? what else to do when tracking is lost?
		ITMLocalScene<ITMVoxel,ITMVoxelIndex> *activeScene = sceneManager->getScene(primarySceneIdx);
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
		ITMLocalScene<ITMVoxel,ITMVoxelIndex> *activeData = sceneManager->getScene(freeviewSceneIdx);
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

