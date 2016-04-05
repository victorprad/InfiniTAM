// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMBasicEngine.h"

#include "../Engines/LowLevel/ITMLowLevelEngineFactory.h"
#include "../Engines/Meshing/ITMMeshingEngineFactory.h"
#include "../Engines/ViewBuilding/ITMViewBuilderFactory.h"
#include "../Engines/Visualisation/ITMVisualisationEngineFactory.h"
#include "../Trackers/ITMTrackerFactory.h"
using namespace ITMLib;

#include "../../ORUtils/NVTimer.h"
#include "../../ORUtils/FileUtils.h"

template <typename TVoxel, typename TIndex>
ITMBasicEngine<TVoxel,TIndex>::ITMBasicEngine(const ITMLibSettings *settings, const ITMRGBDCalib *calib, Vector2i imgSize_rgb, Vector2i imgSize_d)
{
	this->settings = settings;

	// create all the things required for marching cubes and mesh extraction
	// - uses additional memory (lots!)
	static const bool createMeshingEngine = true;

	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

	Vector2i paddingSize(0, 0);

	imgSize_d += 2 * paddingSize; imgSize_rgb += 2 * paddingSize;

	MemoryDeviceType memoryType = settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
	this->scene = new ITMScene<TVoxel,TIndex>(&settings->sceneParams, settings->useSwapping, memoryType);

	const ITMLibSettings::DeviceType deviceType = settings->deviceType;

	lowLevelEngine = ITMLowLevelEngineFactory::MakeLowLevelEngine(deviceType);
	viewBuilder = ITMViewBuilderFactory::MakeViewBuilder(calib, deviceType, paddingSize);
	visualisationEngine = ITMVisualisationEngineFactory::MakeVisualisationEngine<TVoxel,TIndex>(deviceType);

	mesh = NULL;
	meshingEngine = NULL;
	if (createMeshingEngine)
	{
		mesh = new ITMMesh(memoryType);
		meshingEngine = ITMMeshingEngineFactory::MakeMeshingEngine<TVoxel,TIndex>(deviceType);
	}

	denseMapper = new ITMDenseMapper<TVoxel,TIndex>(settings);
	denseMapper->ResetScene(scene);

	imuCalibrator = new ITMIMUCalibrator_iPad();
	tracker = ITMTrackerFactory<TVoxel,TIndex>::Instance().Make(imgSize_rgb, imgSize_d, settings, lowLevelEngine, imuCalibrator, scene);
	trackingController = new ITMTrackingController(tracker, settings);

	Vector2i trackedImageSize = trackingController->GetTrackedImageSize(imgSize_rgb, imgSize_d);

	renderState_live = visualisationEngine->CreateRenderState(scene, trackedImageSize);
	renderState_freeview = NULL; //will be created by the visualisation engine

	trackingState = new ITMTrackingState(trackedImageSize, memoryType);
	tracker->UpdateInitialPose(trackingState);

	view = NULL; // will be allocated by the view builder
	
	relocaliser = new RelocLib::Relocaliser(imgSize_d, Vector2f(settings->sceneParams.viewFrustum_min, settings->sceneParams.viewFrustum_max), 0.2f, 500, 4);
	kfRaycast = new ITMUChar4Image(imgSize_d, memoryType);

	trackingActive = true;
	fusionActive = true;
	mainProcessingActive = true;
	trackingInitialised = false;
	relocalisationCount = 0;
	framesProcessed = 0;
}

template <typename TVoxel, typename TIndex>
ITMBasicEngine<TVoxel,TIndex>::~ITMBasicEngine()
{
	delete renderState_live;
	if (renderState_freeview!=NULL) delete renderState_freeview;

	delete scene;

	delete denseMapper;
	delete trackingController;

	delete tracker;
	delete imuCalibrator;

	delete lowLevelEngine;
	delete viewBuilder;

	delete trackingState;
	if (view != NULL) delete view;

	delete visualisationEngine;

	delete relocaliser;
	delete kfRaycast;

	if (meshingEngine != NULL) delete meshingEngine;

	if (mesh != NULL) delete mesh;
}

template <typename TVoxel, typename TIndex>
ITMMesh* ITMBasicEngine<TVoxel,TIndex>::UpdateMesh(void)
{
	if (mesh != NULL) meshingEngine->MeshScene(mesh, scene);
	return mesh;
}

template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::SaveSceneToMesh(const char *objFileName)
{
	if (mesh == NULL) return;
	meshingEngine->MeshScene(mesh, scene);
	mesh->WriteSTL(objFileName);
}

static int QuaternionFromRotationMatrix_variant(const double *matrix)
{
	int variant = 0;
	if
		((matrix[4]>-matrix[8]) && (matrix[0]>-matrix[4]) && (matrix[0]>-matrix[8]))
	{
		variant = 0;
	}
	else if ((matrix[4]<-matrix[8]) && (matrix[0]>
		matrix[4]) && (matrix[0]> matrix[8])) {
		variant = 1;
	}
	else if ((matrix[4]> matrix[8]) && (matrix[0]<
		matrix[4]) && (matrix[0]<-matrix[8])) {
		variant = 2;
	}
	else if ((matrix[4]<
		matrix[8]) && (matrix[0]<-matrix[4]) && (matrix[0]< matrix[8])) {
		variant = 3;
	}
	return variant;
}

void QuaternionFromRotationMatrix(const double *matrix, double *q) {
	/* taken from "James Diebel. Representing Attitude: Euler
	Angles, Quaternions, and Rotation Vectors. Technical Report, Stanford
	University, Palo Alto, CA."
	*/

	// choose the numerically best variant...
	int variant = QuaternionFromRotationMatrix_variant(matrix);
	double denom = 1.0;
	if (variant == 0) {
		denom += matrix[0] + matrix[4] + matrix[8];
	}
	else {
		int tmp = variant * 4;
		denom += matrix[tmp - 4];
		denom -= matrix[tmp % 12];
		denom -= matrix[(tmp + 4) % 12];
	}
	denom = sqrt(denom);
	q[variant] = 0.5*denom;

	denom *= 2.0;
	switch (variant) {
	case 0:
		q[1] = (matrix[5] - matrix[7]) / denom;
		q[2] = (matrix[6] - matrix[2]) / denom;
		q[3] = (matrix[1] - matrix[3]) / denom;
		break;
	case 1:
		q[0] = (matrix[5] - matrix[7]) / denom;
		q[2] = (matrix[1] + matrix[3]) / denom;
		q[3] = (matrix[6] + matrix[2]) / denom;
		break;
	case 2:
		q[0] = (matrix[6] - matrix[2]) / denom;
		q[1] = (matrix[1] + matrix[3]) / denom;
		q[3] = (matrix[5] + matrix[7]) / denom;
		break;
	case 3:
		q[0] = (matrix[1] - matrix[3]) / denom;
		q[1] = (matrix[6] + matrix[2]) / denom;
		q[2] = (matrix[5] + matrix[7]) / denom;
		break;
	}

	if (q[0] < 0.0f) for (int i = 0; i < 4; ++i) q[i] *= -1.0f;
}

template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement)
{
	// prepare image and turn it into a depth image
	bool modelSensorNoise = tracker->requiresDepthReliability();
	if (imuMeasurement == NULL) viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, modelSensorNoise);
	else viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, imuMeasurement);

	if (!mainProcessingActive) return;

	// tracking
	ORUtils::SE3Pose oldPose(*(trackingState->pose_d));
	if (trackingActive) trackingController->Track(trackingState, view);

	int trackingSuccess = 0;
	if (trackingState->poseQuality > settings->goodTrackingThreshold) trackingSuccess = 2;
	else if (trackingState->poseQuality > settings->poorTrackingThreshold) trackingSuccess = 1;

	if (!settings->useTrackingFailureDetection) trackingSuccess = 2;

	static int trackingSuccess_prev = -1;
	if (trackingSuccess != trackingSuccess_prev) {
		if (trackingSuccess == 2) fprintf(stderr, "tracking good\n");
		if (trackingSuccess == 1) fprintf(stderr, "tracking poor\n");
		if (trackingSuccess == 0) fprintf(stderr, "tracking LOST\n");
		trackingSuccess_prev = trackingSuccess;
	}

	//relocalisation
	int addKeyframeIdx = -1;
	if (settings->useRelocalisation && settings->useTrackingFailureDetection) {
		if (trackingSuccess == 2 && relocalisationCount > 0) relocalisationCount--;

		int NN; float distances;
		addKeyframeIdx = relocaliser->ProcessFrame(view->depth, 1, &NN, &distances, trackingSuccess == 2 && relocalisationCount == 0);
		
		// add keyframe, if necessary
		if (addKeyframeIdx >= 0)
			poseDatabase.storePose(addKeyframeIdx, *(trackingState->pose_d), 0);
		else if (trackingSuccess == 0) {
			relocalisationCount = 10;

			const RelocLib::PoseDatabase::PoseInScene & keyframe = poseDatabase.retrievePose(NN);
			trackingState->pose_d->SetFrom(&keyframe.pose);

			denseMapper->UpdateVisibleList(view, trackingState, scene, renderState_live, true);
			trackingController->Prepare(trackingState, scene, view, visualisationEngine, renderState_live); 
			trackingController->Track(trackingState, view);

			trackingSuccess = 0;
			if (trackingState->poseQuality > settings->goodTrackingThreshold) trackingSuccess = 2;
			else if (trackingState->poseQuality > settings->poorTrackingThreshold) trackingSuccess = 1;
		}
	}

	bool didFusion = false;
	if ((trackingSuccess >= 2 || !trackingInitialised) && (fusionActive) && (relocalisationCount == 0)) {
		// fusion
		denseMapper->ProcessFrame(view, trackingState, scene, renderState_live);
		didFusion = true;
		if (framesProcessed > 50) trackingInitialised = true;

		framesProcessed++;
	}

	if (trackingSuccess >= 1) {
		if (!didFusion) denseMapper->UpdateVisibleList(view, trackingState, scene, renderState_live);

		// raycast to renderState_live for tracking and free visualisation
		trackingController->Prepare(trackingState, scene, view, visualisationEngine, renderState_live);

		ORUtils::MemoryBlock<Vector4u>::MemoryCopyDirection memoryCopyDirection = 
			settings->deviceType == ITMLibSettings::DEVICE_CUDA ? ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CUDA : ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU;
		if (addKeyframeIdx >= 0) kfRaycast->SetFrom(renderState_live->raycastImage, memoryCopyDirection);
	}
	else { *trackingState->pose_d = oldPose; }

	const ORUtils::SE3Pose *p = trackingState->pose_d;
	double t[3];
	double R[9];
	double q[4];
	for (int i = 0; i < 3; ++i) t[i] = p->GetInvM().m[3 * 4 + i];
	for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c)
		R[r * 3 + c] = p->GetM().m[c * 4 + r];
	QuaternionFromRotationMatrix(R, q);
	fprintf(stderr, "%f %f %f %f %f %f %f\n", t[0],
		t[1], t[2], q[1], q[2], q[3], q[0]);

	printf("%d\n", framesProcessed);
}

template <typename TVoxel, typename TIndex>
Vector2i ITMBasicEngine<TVoxel,TIndex>::GetImageSize(void) const
{
	return renderState_live->raycastImage->noDims;
}

template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::GetImage(ITMUChar4Image *out, GetImageType getImageType, ORUtils::SE3Pose *pose, ITMIntrinsics *intrinsics)
{
	if (view == NULL) return;

	out->Clear();

	switch (getImageType)
	{
	case ITMBasicEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
		out->ChangeDims(view->rgb->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) 
			out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	case ITMBasicEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
		out->ChangeDims(view->depth->noDims);
		if (tracker->requiresDepthReliability())
		{
			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depthUncertainty->UpdateHostFromDevice();
			ITMVisualisationEngine<TVoxel, TIndex>::DepthToUchar4(out, view->depthUncertainty);

			WriteToBIN(view->depthUncertainty->GetData(MEMORYDEVICE_CPU), 640 * 480, "c:/temp/frame.bin");
		}
		else
		{
			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depth->UpdateHostFromDevice();
			ITMVisualisationEngine<TVoxel,TIndex>::DepthToUchar4(out, view->depth);
		}

		break;
	case ITMBasicEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
	{
		visualisationEngine->FindVisibleBlocks(scene, trackingState->pose_d, &view->calib->intrinsics_d, renderState_live);
		visualisationEngine->CreateExpectedDepths(scene, trackingState->pose_d, &view->calib->intrinsics_d, renderState_live);
		visualisationEngine->RenderImage(scene, trackingState->pose_d, &view->calib->intrinsics_d, renderState_live, renderState_live->raycastImage, IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL);

		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
			out->SetFrom(renderState_live->raycastImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(renderState_live->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);

		break;
	}
	case ITMBasicEngine::InfiniTAM_IMAGE_SCENERAYCAST:
	{
		ORUtils::Image<Vector4u> *srcImage = NULL;

		if (relocalisationCount != 0) srcImage = kfRaycast;
		else srcImage = renderState_live->raycastImage;

		out->ChangeDims(srcImage->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
			out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);	
		break;
	}
	case ITMBasicEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED:
	case ITMBasicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
	case ITMBasicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
	{
		IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;
		if (getImageType == ITMBasicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME;
		else if (getImageType == ITMBasicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
		if (renderState_freeview == NULL) renderState_freeview = visualisationEngine->CreateRenderState(scene, out->noDims);

		visualisationEngine->FindVisibleBlocks(scene, pose, intrinsics, renderState_freeview);
		visualisationEngine->CreateExpectedDepths(scene, pose, intrinsics, renderState_freeview);
		visualisationEngine->RenderImage(scene, pose, intrinsics, renderState_freeview, renderState_freeview->raycastImage, type);

		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
			out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	}
	case ITMMainEngine::InfiniTAM_IMAGE_UNKNOWN:
		break;
	};
}

template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::turnOnTracking() { trackingActive = true; }

template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::turnOffTracking() { trackingActive = false; }

template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::turnOnIntegration() { fusionActive = true; }

template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::turnOffIntegration() { fusionActive = false; }

template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::turnOnMainProcessing() { mainProcessingActive = true; }

template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel,TIndex>::turnOffMainProcessing() { mainProcessingActive = false; }
