// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMMultiEngine.h"

#include "../Engines/LowLevel/ITMLowLevelEngineFactory.h"
#include "../Engines/ViewBuilding/ITMViewBuilderFactory.h"
#include "../Engines/Visualisation/ITMVisualisationEngineFactory.h"
#include "../Engines/Visualisation/ITMMultiVisualisationEngineFactory.h"
#include "../Trackers/ITMTrackerFactory.h"

#include "../../MiniSlamGraphLib/QuaternionHelpers.h"

using namespace ITMLib;

//#define DEBUG_MULTISCENE

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
static const int k_loopcloseneighbours = 3;
// maximum distance reported by LCD library to attempt relocalisation
static const float F_maxdistattemptreloc = 0.1f;

static const bool MultithreadedGlobalAdjustment = true;

int currentFrameNo = 0;

template <typename TVoxel, typename TIndex>
ITMMultiEngine<TVoxel, TIndex>::ITMMultiEngine(const ITMLibSettings *settings, const ITMRGBDCalib& calib, Vector2i imgSize_rgb, Vector2i imgSize_d)
{
	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

	this->settings = settings;

	const ITMLibSettings::DeviceType deviceType = settings->deviceType;
	lowLevelEngine = ITMLowLevelEngineFactory::MakeLowLevelEngine(deviceType);
	viewBuilder = ITMViewBuilderFactory::MakeViewBuilder(calib, deviceType);
	visualisationEngine = ITMVisualisationEngineFactory::MakeVisualisationEngine<TVoxel, TIndex>(deviceType);

	renderState_freeview = NULL; //will be created by the visualisation engine

	denseMapper = new ITMDenseMapper<TVoxel, TIndex>(settings);

	imuCalibrator = new ITMIMUCalibrator_iPad();
	tracker = ITMTrackerFactory::Instance().Make(imgSize_rgb, imgSize_d, settings, lowLevelEngine, imuCalibrator, &settings->sceneParams);
	trackingController = new ITMTrackingController(tracker, settings);
	trackedImageSize = trackingController->GetTrackedImageSize(imgSize_rgb, imgSize_d);

	freeviewSceneIdx = 0;
	mSceneManager = new ITMMultiSceneManager_instance<TVoxel, TIndex>(settings, visualisationEngine, denseMapper, trackedImageSize);
	mActiveDataManager = new ITMActiveSceneManager(mSceneManager);
	mActiveDataManager->initiateNewScene(true);

	//TODO	tracker->UpdateInitialPose(allData[0]->trackingState);

	view = NULL; // will be allocated by the view builder

	//mLoopClosureDetector = new LCDLib::LoopClosureDetector(imgSize_d, Vector2f(0.3f,5.0f), 0.15f, 4000, 8);
	mLoopClosureDetector = new RelocLib::Relocaliser(imgSize_d, Vector2f(0.3f, 5.0f), 0.1f, 1000, 4);
	//mLoopClosureDetector = new LCDLib::LoopClosureDetector(imgSize_d, Vector2f(0.3f,4.0f), 0.1f, 2000, 6);
	mGlobalAdjustmentEngine = new ITMGlobalAdjustmentEngine();
	mScheduleGlobalAdjustment = false;
	if (MultithreadedGlobalAdjustment) mGlobalAdjustmentEngine->startSeparateThread();

	multiVisualisationEngine = ITMMultiVisualisationEngineFactory::MakeVisualisationEngine<TVoxel,TIndex>(deviceType);
	renderState_multiscene = NULL;
}

template <typename TVoxel, typename TIndex>
ITMMultiEngine<TVoxel, TIndex>::~ITMMultiEngine(void)
{
	//delete multiVisualisationEngine;
	if (renderState_multiscene != NULL) delete renderState_multiscene;

	delete mGlobalAdjustmentEngine;
	delete mActiveDataManager;
	delete mSceneManager;

	if (renderState_freeview != NULL) delete renderState_freeview;

	delete denseMapper;
	delete trackingController;

	delete tracker;
	delete imuCalibrator;

	delete lowLevelEngine;
	delete viewBuilder;

	if (view != NULL) delete view;

	delete visualisationEngine;

	delete mLoopClosureDetector;

	delete multiVisualisationEngine;
}

template <typename TVoxel, typename TIndex>
void ITMMultiEngine<TVoxel, TIndex>::changeFreeviewSceneIdx(ORUtils::SE3Pose *pose, int newIdx)
{
	//	if ((newIdx < 0)||((unsigned)newIdx >= sceneManager->numScenes())) return;
	if (newIdx < -1) newIdx = (int)mSceneManager->numScenes() - 1;
	if ((unsigned)newIdx >= mSceneManager->numScenes()) newIdx = -1;

	ORUtils::SE3Pose trafo = mSceneManager->findTransformation(freeviewSceneIdx, newIdx);
	pose->SetM(pose->GetM() * trafo.GetInvM());
	pose->Coerce();
	freeviewSceneIdx = newIdx;
}

template <typename TVoxel, typename TIndex>
ITMTrackingState* ITMMultiEngine<TVoxel, TIndex>::GetTrackingState(void)
{
	int idx = mActiveDataManager->findPrimarySceneIdx();
	if (idx < 0) idx = 0;
	return mSceneManager->getScene(idx)->trackingState;
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


template <typename TVoxel, typename TIndex>
ITMTrackingState::TrackingResult ITMMultiEngine<TVoxel, TIndex>::ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement)
{
	// prepare image and turn it into a depth image
	if (imuMeasurement == NULL) viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter);
	else viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, imuMeasurement);

	std::vector<TodoListEntry> todoList;

	// find primary data, if available
	int primaryDataIdx = mActiveDataManager->findPrimaryDataIdx();

	// if there is a "primary data index", process it
	if (primaryDataIdx >= 0) {
		todoList.push_back(TodoListEntry(primaryDataIdx, true, true, true));
	}

	// after primary scene, make sure to process all relocalisations, new
	// scenes and loop closures
	for (int i = 0; i < mActiveDataManager->numActiveScenes(); ++i) {
		if (mActiveDataManager->getSceneType(i) == ITMActiveSceneManager::NEW_SCENE) todoList.push_back(TodoListEntry(i, true, true, true));
		else if (mActiveDataManager->getSceneType(i) == ITMActiveSceneManager::LOOP_CLOSURE) todoList.push_back(TodoListEntry(i, true, false, true));
		else if (mActiveDataManager->getSceneType(i) == ITMActiveSceneManager::RELOCALISATION) todoList.push_back(TodoListEntry(i, true, false, true));
	}

	// finally, once all is done, call the loop closure detection engine
	todoList.push_back(TodoListEntry(-1, false, false, false));

#ifdef DEBUG_MULTISCENE
	fprintf(stderr, "tracking data blocks:");
#endif
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
#ifdef DEBUG_MULTISCENE
			fprintf(stderr, " LCD(%i)", primaryTrackingSuccess);
#endif
			int NN[k_loopcloseneighbours];
			float distances[k_loopcloseneighbours];
			view->depth->UpdateHostFromDevice();
			int addKeyframeIdx = mLoopClosureDetector->ProcessFrame(view->depth, k_loopcloseneighbours, NN, distances, primaryTrackingSuccess);
			int primarySceneIdx = -1;
			if (primaryDataIdx >= 0) primarySceneIdx = mActiveDataManager->getSceneIndex(primaryDataIdx);

			// add keyframe, if necessary
			if (addKeyframeIdx >= 0)
			{
#ifdef DEBUG_MULTISCENE
				fprintf(stderr, "add keyframe\n");
#endif
				mPoseDatabase.storePose(addKeyframeIdx, *(mSceneManager->getScene(primarySceneIdx)->trackingState->pose_d), primarySceneIdx);
			}
			else  for (int j = 0; j < k_loopcloseneighbours; ++j)
			{
				if (distances[j] > F_maxdistattemptreloc) continue;
				const RelocLib::PoseDatabase::PoseInScene & keyframe = mPoseDatabase.retrievePose(NN[j]);
				int newDataIdx = mActiveDataManager->initiateNewLink(keyframe.sceneIdx, keyframe.pose, (primarySceneIdx < 0));
				//if (currentFrameNo > 3640)
					//newDataIdx
				if (newDataIdx >= 0) {
					// this is a new relocalisation attempt
					TodoListEntry todoItem(newDataIdx, true, false, true);
					todoItem.preprepare = true;
					todoList.push_back(todoItem);
				}
			}
			continue;
		}

		ITMLocalScene<TVoxel, TIndex> *currentScene = NULL;
		int currentSceneIdx = mActiveDataManager->getSceneIndex(todoList[i].dataID);
		currentScene = mSceneManager->getScene(currentSceneIdx);

		// if a new relocalisation/loopclosure is started, this will
		// do the initial raycasting before tracking can start
		if (todoList[i].preprepare) {
			denseMapper->UpdateVisibleList(view, currentScene->trackingState, currentScene->scene, currentScene->renderState);
			trackingController->Prepare(currentScene->trackingState, currentScene->scene, view, visualisationEngine, currentScene->renderState);
		}

		//fprintf(stderr, "scene %i: ", currentSceneIdx);
				// tracking
		if (todoList[i].track)
		{
			int dataID = todoList[i].dataID;
#ifdef DEBUG_MULTISCENE
			int blocksInUse = currentScene->scene->index.getNumAllocatedVoxelBlocks() - currentScene->scene->localVBA.lastFreeBlockId - 1;
			fprintf(stderr, " %i%s (%i)", currentSceneIdx, (todoList[i].dataID == primaryDataIdx) ? "*" : "", blocksInUse);
#endif

			ORUtils::SE3Pose oldPose(*(currentScene->trackingState->pose_d));
			trackingController->Track(currentScene->trackingState, view);

			int trackingSuccess = 0;
			if (currentScene->trackingState->trackerResult == ITMTrackingState::TRACKING_GOOD) trackingSuccess = 2;
			else if (currentScene->trackingState->trackerResult == ITMTrackingState::TRACKING_POOR) trackingSuccess = 1;

			if (currentScene->trackingState->trackerResult == ITMTrackingState::TRACKING_FAILED)
				printf("tracking failed\n");

			if (trackingSuccess < 2)
			{
				todoList[i].fusion = false;
			}
			if (trackingSuccess < 1)
			{
				todoList[i].prepare = false;
				*(currentScene->trackingState->pose_d) = oldPose;
			}

			{
				static int framecounter = 0;
				if (framecounter++ < 3) todoList[i].fusion = true;
			}
			// TODO: mark some more scenes as "LOST" if they are lost... possibly...

			if (mActiveDataManager->getSceneType(dataID) == ITMActiveSceneManager::PRIMARY_SCENE)
			{
				if (trackingSuccess >= 2) primaryTrackingSuccess = true;

				else if (trackingSuccess < 1)
				{
					primaryDataIdx = -1;
					todoList.resize(i + 1);
					todoList.push_back(TodoListEntry(-1, false, false, false));
#ifdef DEBUG_MULTISCENE
					fprintf(stderr, "Lost track of primary scene\n");
#endif
				}
			}
			mActiveDataManager->recordTrackingResult(dataID, trackingSuccess, primaryTrackingSuccess);
		}

		// fusion
		if (todoList[i].fusion) {
			denseMapper->ProcessFrame(view, currentScene->trackingState, currentScene->scene, currentScene->renderState);
		}
		else if (todoList[i].prepare) {
			denseMapper->UpdateVisibleList(view, currentScene->trackingState, currentScene->scene, currentScene->renderState);
		}

		// raycast to renderState_live for tracking and free visualisation
		if (todoList[i].prepare) {
			trackingController->Prepare(currentScene->trackingState, currentScene->scene, view, visualisationEngine, currentScene->renderState);
		}
	}

	mScheduleGlobalAdjustment |= mActiveDataManager->maintainActiveData();

	if (mScheduleGlobalAdjustment) {
		if (mGlobalAdjustmentEngine->updateMeasurements(*mSceneManager)) {
			if (MultithreadedGlobalAdjustment) mGlobalAdjustmentEngine->wakeupSeparateThread();
			else mGlobalAdjustmentEngine->runGlobalAdjustment();
			mScheduleGlobalAdjustment = false;
		}
	}
	mGlobalAdjustmentEngine->retrieveNewEstimates(*mSceneManager);

	//	printf("currentFrame = %d\n", currentFrameNo);
	//	currentFrameNo++;
	//
	//{
	//fprintf(stderr, "submap sizes:");
	//for (int s = 0; s < mSceneManager->numScenes(); ++s) {
	//fprintf(stderr, " %i", mSceneManager->getSceneSize(s));
	//}
	//fprintf(stderr, "\n");
	//}

#ifdef DEBUG_MULTISCENE
	fprintf(stderr, "...done!\n");
#endif

	return ITMTrackingState::TRACKING_GOOD;
}

//template <typename TVoxel, typename TIndex>
//void ITMMultiEngine<TVoxel,TIndex>::SaveSceneToMesh(const char *objFileName)
//{
//	// TODO: this will all fail without CUDA...
//	ITMMesh *mesh = new ITMMesh(MEMORYDEVICE_CUDA, SDF_LOCAL_BLOCK_NUM * 32 * 16);
//	ITMMultiMeshingEngine_CUDA<TVoxel,TIndex> *meshingEngine = new ITMMultiMeshingEngine_CUDA<TVoxel,TIndex>();
//	meshingEngine->MeshScene(mesh, *mSceneManager);
//	mesh->WriteSTL(objFileName);
//	delete meshingEngine;
//	delete mesh;
//}

template <typename TVoxel, typename TIndex>
Vector2i ITMMultiEngine<TVoxel, TIndex>::GetImageSize(void) const
{
	return trackedImageSize;
}

template <typename TVoxel, typename TIndex>
void ITMMultiEngine<TVoxel, TIndex>::GetImage(ITMUChar4Image *out, GetImageType getImageType, ORUtils::SE3Pose *pose, ITMIntrinsics *intrinsics)
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
		ITMVisualisationEngine<TVoxel, TIndex>::DepthToUchar4(out, view->depth);
		break;
	case ITMMultiEngine::InfiniTAM_IMAGE_SCENERAYCAST:
	case ITMMultiEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
	case ITMMultiEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE:
	{
		int visualisationSceneIdx = mActiveDataManager->findBestVisualisationSceneIdx();
		if (visualisationSceneIdx < 0) break; // TODO: clear image? what else to do when tracking is lost?

		ITMLocalScene<TVoxel, TIndex> *activeScene = mSceneManager->getScene(visualisationSceneIdx);

		IITMVisualisationEngine::RenderRaycastSelection raycastType;
		if (activeScene->trackingState->age_pointCloud <= 0) raycastType = IITMVisualisationEngine::RENDER_FROM_OLD_RAYCAST;
		else raycastType = IITMVisualisationEngine::RENDER_FROM_OLD_FORWARDPROJ;

		IITMVisualisationEngine::RenderImageType imageType;
		switch (getImageType) {
		case ITMMultiEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE:
			imageType = IITMVisualisationEngine::RENDER_COLOUR_FROM_CONFIDENCE;
			break;
		case ITMMultiEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
			imageType = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
			break;
		default:
			imageType = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE_IMAGENORMALS;
		}

		visualisationEngine->RenderImage(activeScene->scene, activeScene->trackingState->pose_d, &view->calib.intrinsics_d, activeScene->renderState, activeScene->renderState->raycastImage, imageType, raycastType);

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
	case ITMMultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE:
	{
		IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;
		if (getImageType == ITMMultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME;
		else if (getImageType == ITMMultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
		else if (getImageType == ITMMultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_CONFIDENCE;

		if (freeviewSceneIdx >= 0) {
			ITMLocalScene<TVoxel, TIndex> *activeData = mSceneManager->getScene(freeviewSceneIdx);
			if (renderState_freeview == NULL) renderState_freeview = visualisationEngine->CreateRenderState(activeData->scene, out->noDims);

			visualisationEngine->FindVisibleBlocks(activeData->scene, pose, intrinsics, renderState_freeview);
			visualisationEngine->CreateExpectedDepths(activeData->scene, pose, intrinsics, renderState_freeview);
			visualisationEngine->RenderImage(activeData->scene, pose, intrinsics, renderState_freeview, renderState_freeview->raycastImage, type);

			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
				out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
			else out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		}
		else {
			if (renderState_multiscene == NULL) renderState_multiscene = multiVisualisationEngine->CreateRenderState(mSceneManager->getScene(0)->scene, out->noDims);
			multiVisualisationEngine->PrepareRenderState(*mSceneManager, renderState_multiscene);
			multiVisualisationEngine->CreateExpectedDepths(pose, intrinsics, renderState_multiscene);
			multiVisualisationEngine->RenderImage(pose, intrinsics, renderState_multiscene, renderState_multiscene->raycastImage, type);
			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
				out->SetFrom(renderState_multiscene->raycastImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
			else out->SetFrom(renderState_multiscene->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		}

		break;
	}
	case ITMMultiEngine::InfiniTAM_IMAGE_UNKNOWN:
		break;
	};
}