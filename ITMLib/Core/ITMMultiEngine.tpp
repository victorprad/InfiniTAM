// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMMultiEngine.h"

#include "../Engines/LowLevel/ITMLowLevelEngineFactory.h"
#include "../Engines/ViewBuilding/ITMViewBuilderFactory.h"
#include "../Engines/Visualisation/ITMVisualisationEngineFactory.h"
#include "../Engines/Visualisation/ITMMultiVisualisationEngineFactory.h"
#include "../Trackers/ITMTrackerFactory.h"

#include "../../MiniSlamGraphLib/QuaternionHelpers.h"

using namespace ITMLib;

//#define DEBUG_MULTISCENE

// number of nearest neighbours to find in the loop closure detection
static const int k_loopcloseneighbours = 1;

// maximum distance reported by LCD library to attempt relocalisation
static const float F_maxdistattemptreloc = 0.05f;

// loop closure global adjustment runs on a separate thread
static const bool separateThreadGlobalAdjustment = true;

template <typename TVoxel, typename TIndex>
ITMMultiEngine<TVoxel, TIndex>::ITMMultiEngine(const ITMLibSettings *settings, const ITMRGBDCalib& calib, Vector2i imgSize_rgb, Vector2i imgSize_d)
{
	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

	this->settings = settings;

	const ITMLibSettings::DeviceType deviceType = settings->deviceType;
	lowLevelEngine = ITMLowLevelEngineFactory::MakeLowLevelEngine(deviceType);
	viewBuilder = ITMViewBuilderFactory::MakeViewBuilder(calib, deviceType);
	visualisationEngine = ITMVisualisationEngineFactory::MakeVisualisationEngine<TVoxel, TIndex>(deviceType);

	meshingEngine = NULL;
	if (settings->createMeshingEngine)
		meshingEngine = ITMMultiMeshingEngineFactory::MakeMeshingEngine<TVoxel, TIndex>(deviceType);

	renderState_freeview = NULL; //will be created by the visualisation engine

	denseMapper = new ITMDenseMapper<TVoxel, TIndex>(settings);

	imuCalibrator = new ITMIMUCalibrator_iPad();
	tracker = ITMTrackerFactory::Instance().Make(imgSize_rgb, imgSize_d, settings, lowLevelEngine, imuCalibrator, &settings->sceneParams);
	trackingController = new ITMTrackingController(tracker, settings);
	trackedImageSize = trackingController->GetTrackedImageSize(imgSize_rgb, imgSize_d);

	freeviewLocalMapIdx = 0;
	mapManager = new ITMVoxelMapGraphManager<TVoxel, TIndex>(settings, visualisationEngine, denseMapper, trackedImageSize);
	mActiveDataManager = new ITMActiveMapManager(mapManager);
	mActiveDataManager->initiateNewLocalMap(true);

	//TODO	tracker->UpdateInitialPose(allData[0]->trackingState);

	view = NULL; // will be allocated by the view builder

	relocaliser = new FernRelocLib::Relocaliser<float>(imgSize_d, Vector2f(settings->sceneParams.viewFrustum_min, settings->sceneParams.viewFrustum_max), 0.1f, 1000, 4);

	mGlobalAdjustmentEngine = new ITMGlobalAdjustmentEngine();
	mScheduleGlobalAdjustment = false;
	if (separateThreadGlobalAdjustment) mGlobalAdjustmentEngine->startSeparateThread();

	multiVisualisationEngine = ITMMultiVisualisationEngineFactory::MakeVisualisationEngine<TVoxel,TIndex>(deviceType);
	renderState_multiscene = NULL;
}

template <typename TVoxel, typename TIndex>
ITMMultiEngine<TVoxel, TIndex>::~ITMMultiEngine(void)
{
	if (renderState_multiscene != NULL) delete renderState_multiscene;

	delete mGlobalAdjustmentEngine;
	delete mActiveDataManager;
	delete mapManager;

	if (renderState_freeview != NULL) delete renderState_freeview;

	delete denseMapper;
	delete trackingController;

	delete tracker;
	delete imuCalibrator;

	delete lowLevelEngine;
	delete viewBuilder;

	if (view != NULL) delete view;

	delete visualisationEngine;

	delete relocaliser;

	delete multiVisualisationEngine;
}

template <typename TVoxel, typename TIndex>
void ITMMultiEngine<TVoxel, TIndex>::changeFreeviewLocalMapIdx(ORUtils::SE3Pose *pose, int newIdx)
{
	//if ((newIdx < 0) || ((unsigned)newIdx >= mapManager->numLocalMaps())) return;

	if (newIdx < -1) newIdx = (int)mapManager->numLocalMaps() - 1;
	if ((unsigned)newIdx >= mapManager->numLocalMaps()) newIdx = -1;

	ORUtils::SE3Pose trafo = mapManager->findTransformation(freeviewLocalMapIdx, newIdx);
	pose->SetM(pose->GetM() * trafo.GetInvM());
	pose->Coerce();
	freeviewLocalMapIdx = newIdx;
}

template <typename TVoxel, typename TIndex>
ITMTrackingState* ITMMultiEngine<TVoxel, TIndex>::GetTrackingState(void)
{
	int idx = mActiveDataManager->findPrimaryLocalMapIdx();
	if (idx < 0) idx = 0;
	return mapManager->getLocalMap(idx)->trackingState;
}

// -whenever a new local scene is added, add to list of "to be established 3D relations"
// - whenever a relocalisation is detected, add to the same list, preserving any existing information on that 3D relation
//
// - for all 3D relations to be established :
// -attempt tracking in both scenes
// - if success, add to list of new candidates
// - if less than n_overlap "new candidates" in more than n_reloctrialframes frames, discard
// - if at least n_overlap "new candidates" :
// 	- try to compute 3D relation, weighting old information accordingly
//	- if outlier ratio below p_relation_outliers and at least n_overlap inliers, success

struct TodoListEntry {
	TodoListEntry(int _activeDataID, bool _track, bool _fusion, bool _prepare)
		: dataId(_activeDataID), track(_track), fusion(_fusion), prepare(_prepare), preprepare(false) {}
	TodoListEntry(void) {}
	int dataId;
	bool track;
	bool fusion;
	bool prepare;
	bool preprepare;
};

template <typename TVoxel, typename TIndex>
ITMTrackingState::TrackingResult ITMMultiEngine<TVoxel, TIndex>::ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement)
{
	std::vector<TodoListEntry> todoList;
	ITMTrackingState::TrackingResult primaryLocalMapTrackingResult;

	// prepare image and turn it into a depth image
	if (imuMeasurement == NULL) viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter);
	else viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, imuMeasurement);

	// find primary data, if available
	int primaryDataIdx = mActiveDataManager->findPrimaryDataIdx();

	// if there is a "primary data index", process it
	if (primaryDataIdx >= 0) todoList.push_back(TodoListEntry(primaryDataIdx, true, true, true));

	// after primary local map, make sure to process all relocalisations, new scenes and loop closures
	for (int i = 0; i < mActiveDataManager->numActiveLocalMaps(); ++i)
	{
		switch (mActiveDataManager->getLocalMapType(i))
		{
		case ITMActiveMapManager::NEW_LOCAL_MAP: todoList.push_back(TodoListEntry(i, true, true, true));
		case ITMActiveMapManager::LOOP_CLOSURE: todoList.push_back(TodoListEntry(i, true, false, true));
		case ITMActiveMapManager::RELOCALISATION: todoList.push_back(TodoListEntry(i, true, false, true));
		default: break;
		}
	}

	// finally, once all is done, call the loop closure detection engine
	todoList.push_back(TodoListEntry(-1, false, false, false));

	bool primaryTrackingSuccess = false;
	for (size_t i = 0; i < todoList.size(); ++i)
	{
		// - first pass of the todo list is for primary local map and ongoing relocalisation and loopclosure attempts
		// - an element with id -1 marks the end of the first pass, a request to call the loop closure detection engine, and
		//   the start of the second pass
		// - second tracking pass will be about newly detected loop closures, relocalisations, etc.

		if (todoList[i].dataId == -1)
		{
#ifdef DEBUG_MULTISCENE
			fprintf(stderr, " Reloc(%i)", primaryTrackingSuccess);
#endif
			int NN[k_loopcloseneighbours]; float distances[k_loopcloseneighbours];
			view->depth->UpdateHostFromDevice();

			//primary map index
			int primaryLocalMapIdx = -1;
			if (primaryDataIdx >= 0) primaryLocalMapIdx = mActiveDataManager->getLocalMapIndex(primaryDataIdx);

			//check if relocaliser has fired
			ORUtils::SE3Pose *pose = primaryLocalMapIdx >= 0 ? mapManager->getLocalMap(primaryLocalMapIdx)->trackingState->pose_d : NULL;
			bool hasAddedKeyframe = relocaliser->ProcessFrame(view->depth, pose, primaryLocalMapIdx, k_loopcloseneighbours, NN, distances, primaryTrackingSuccess);

			//frame not added and tracking failed -> we need to relocalise
			if (!hasAddedKeyframe)
			{
				for (int j = 0; j < k_loopcloseneighbours; ++j)
				{
					if (distances[j] > F_maxdistattemptreloc) continue;
					const FernRelocLib::PoseDatabase::PoseInScene & keyframe = relocaliser->RetrievePose(NN[j]);
					int newDataIdx = mActiveDataManager->initiateNewLink(keyframe.sceneIdx, keyframe.pose, (primaryLocalMapIdx < 0));
					if (newDataIdx >= 0)
					{
						TodoListEntry todoItem(newDataIdx, true, false, true);
						todoItem.preprepare = true;
						todoList.push_back(todoItem);
					}
				}
			}

			continue;
		}

		ITMLocalMap<TVoxel, TIndex> *currentLocalMap = NULL;
		int currentLocalMapIdx = mActiveDataManager->getLocalMapIndex(todoList[i].dataId);
		currentLocalMap = mapManager->getLocalMap(currentLocalMapIdx);

		// if a new relocalisation/loopclosure is started, this will do the initial raycasting before tracking can start
		if (todoList[i].preprepare) 
		{
			denseMapper->UpdateVisibleList(view, currentLocalMap->trackingState, currentLocalMap->scene, currentLocalMap->renderState);
			trackingController->Prepare(currentLocalMap->trackingState, currentLocalMap->scene, view, visualisationEngine, currentLocalMap->renderState);
		}

		if (todoList[i].track)
		{
			int dataId = todoList[i].dataId;

#ifdef DEBUG_MULTISCENE
			int blocksInUse = currentLocalMap->scene->index.getNumAllocatedVoxelBlocks() - currentLocalMap->scene->localVBA.lastFreeBlockId - 1;
			fprintf(stderr, " %i%s (%i)", currentLocalMapIdx, (todoList[i].dataId == primaryDataIdx) ? "*" : "", blocksInUse);
#endif

			// actual tracking
			ORUtils::SE3Pose oldPose(*(currentLocalMap->trackingState->pose_d));
			trackingController->Track(currentLocalMap->trackingState, view);

			// tracking is allowed to be poor only in the primary scenes. 
			ITMTrackingState::TrackingResult trackingResult = currentLocalMap->trackingState->trackerResult;
			if (mActiveDataManager->getLocalMapType(dataId) != ITMActiveMapManager::PRIMARY_LOCAL_MAP)
				if (trackingResult == ITMTrackingState::TRACKING_POOR) trackingResult = ITMTrackingState::TRACKING_FAILED;

			// actions on tracking result for all scenes TODO: incorporate behaviour on tracking failure from settings
			if (trackingResult != ITMTrackingState::TRACKING_GOOD) todoList[i].fusion = false;

			if (trackingResult == ITMTrackingState::TRACKING_FAILED)
			{
				todoList[i].prepare = false;
				*(currentLocalMap->trackingState->pose_d) = oldPose;
			}

			// actions on tracking result for primary local map
			if (mActiveDataManager->getLocalMapType(dataId) == ITMActiveMapManager::PRIMARY_LOCAL_MAP)
			{
				primaryLocalMapTrackingResult = trackingResult;

				if (trackingResult == ITMTrackingState::TRACKING_GOOD) primaryTrackingSuccess = true;

				// we need to relocalise in the primary local map
				else if (trackingResult == ITMTrackingState::TRACKING_FAILED)
				{
					primaryDataIdx = -1;
					todoList.resize(i + 1);
					todoList.push_back(TodoListEntry(-1, false, false, false));
				}
			}

			mActiveDataManager->recordTrackingResult(dataId, trackingResult, primaryTrackingSuccess);
		}

		// fusion in any subscene as long as tracking is good for the respective subscene
		if (todoList[i].fusion) denseMapper->ProcessFrame(view, currentLocalMap->trackingState, currentLocalMap->scene, currentLocalMap->renderState);
		else if (todoList[i].prepare) denseMapper->UpdateVisibleList(view, currentLocalMap->trackingState, currentLocalMap->scene, currentLocalMap->renderState);

		// raycast to renderState_live for tracking and free visualisation
		if (todoList[i].prepare) trackingController->Prepare(currentLocalMap->trackingState, currentLocalMap->scene, view, visualisationEngine, currentLocalMap->renderState);
	}

	mScheduleGlobalAdjustment |= mActiveDataManager->maintainActiveData();

	if (mScheduleGlobalAdjustment) 
	{
		if (mGlobalAdjustmentEngine->updateMeasurements(*mapManager)) 
		{
			if (separateThreadGlobalAdjustment) mGlobalAdjustmentEngine->wakeupSeparateThread();
			else mGlobalAdjustmentEngine->runGlobalAdjustment();

			mScheduleGlobalAdjustment = false;
		}
	}
	mGlobalAdjustmentEngine->retrieveNewEstimates(*mapManager);

	return primaryLocalMapTrackingResult;
}

template <typename TVoxel, typename TIndex>
void ITMMultiEngine<TVoxel, TIndex>::SaveSceneToMesh(const char *modelFileName)
{
	if (meshingEngine == NULL) return;

	ITMMesh *mesh = new ITMMesh(settings->GetMemoryType());

	meshingEngine->MeshScene(mesh, *mapManager);
	mesh->WriteSTL(modelFileName);
	
	delete mesh;
}

template <typename TVoxel, typename TIndex>
void ITMMultiEngine<TVoxel, TIndex>::SaveToFile()
{

}

template <typename TVoxel, typename TIndex>
void ITMMultiEngine<TVoxel, TIndex>::LoadFromFile()
{

}

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
    case ITMMultiEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME: //TODO: add colour rendering
	case ITMMultiEngine::InfiniTAM_IMAGE_SCENERAYCAST:
	case ITMMultiEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
	case ITMMultiEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE:
	{
		int visualisationLocalMapIdx = mActiveDataManager->findBestVisualisationLocalMapIdx();
		if (visualisationLocalMapIdx < 0) break; // TODO: clear image? what else to do when tracking is lost?

		ITMLocalMap<TVoxel, TIndex> *activeLocalMap = mapManager->getLocalMap(visualisationLocalMapIdx);

		IITMVisualisationEngine::RenderRaycastSelection raycastType;
		if (activeLocalMap->trackingState->age_pointCloud <= 0) raycastType = IITMVisualisationEngine::RENDER_FROM_OLD_RAYCAST;
		else raycastType = IITMVisualisationEngine::RENDER_FROM_OLD_FORWARDPROJ;

		IITMVisualisationEngine::RenderImageType imageType;
		switch (getImageType) 
		{
		case ITMMultiEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE:
			imageType = IITMVisualisationEngine::RENDER_COLOUR_FROM_CONFIDENCE;
			break;
		case ITMMultiEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
			imageType = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
			break;
		default:
			imageType = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE_IMAGENORMALS;
		}

		visualisationEngine->RenderImage(activeLocalMap->scene, activeLocalMap->trackingState->pose_d, &view->calib.intrinsics_d, activeLocalMap->renderState, activeLocalMap->renderState->raycastImage, imageType, raycastType);

		ORUtils::Image<Vector4u> *srcImage = activeLocalMap->renderState->raycastImage;
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

		if (freeviewLocalMapIdx >= 0) 
		{
			ITMLocalMap<TVoxel, TIndex> *activeData = mapManager->getLocalMap(freeviewLocalMapIdx);
			if (renderState_freeview == NULL) renderState_freeview = visualisationEngine->CreateRenderState(activeData->scene, out->noDims);

			visualisationEngine->FindVisibleBlocks(activeData->scene, pose, intrinsics, renderState_freeview);
			visualisationEngine->CreateExpectedDepths(activeData->scene, pose, intrinsics, renderState_freeview);
			visualisationEngine->RenderImage(activeData->scene, pose, intrinsics, renderState_freeview, renderState_freeview->raycastImage, type);

			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
				out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
			else out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		}
		else 
		{
			if (renderState_multiscene == NULL) renderState_multiscene = multiVisualisationEngine->CreateRenderState(mapManager->getLocalMap(0)->scene, out->noDims);
			multiVisualisationEngine->PrepareRenderState(*mapManager, renderState_multiscene);
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
