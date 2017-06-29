// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMBasicSurfelEngine.h"

#include "../Engines/LowLevel/ITMLowLevelEngineFactory.h"
#include "../Engines/ViewBuilding/ITMViewBuilderFactory.h"
#include "../Engines/Visualisation/ITMSurfelVisualisationEngineFactory.h"
#include "../Trackers/ITMTrackerFactory.h"

#include "../../ORUtils/NVTimer.h"
#include "../../ORUtils/FileUtils.h"

//#define OUTPUT_TRAJECTORY_QUATERNIONS

using namespace ITMLib;

template <typename TSurfel>
ITMBasicSurfelEngine<TSurfel>::ITMBasicSurfelEngine(const ITMLibSettings *settings, const ITMRGBDCalib& calib, Vector2i imgSize_rgb, Vector2i imgSize_d)
{
	this->settings = settings;

	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

	MemoryDeviceType memoryType = settings->GetMemoryType();
	this->surfelScene = new ITMSurfelScene<TSurfel>(&settings->surfelSceneParams, memoryType);

	const ITMLibSettings::DeviceType deviceType = settings->deviceType;

	lowLevelEngine = ITMLowLevelEngineFactory::MakeLowLevelEngine(deviceType);
	viewBuilder = ITMViewBuilderFactory::MakeViewBuilder(calib, deviceType);
	surfelVisualisationEngine = ITMSurfelVisualisationEngineFactory<TSurfel>::make_surfel_visualisation_engine(deviceType);

	denseSurfelMapper = new ITMDenseSurfelMapper<TSurfel>(imgSize_d, deviceType);
	this->surfelScene->Reset();

	imuCalibrator = new ITMIMUCalibrator_iPad();
	tracker = ITMTrackerFactory::Instance().Make(imgSize_rgb, imgSize_d, settings, lowLevelEngine, imuCalibrator, &settings->sceneParams);
	trackingController = new ITMTrackingController(tracker, settings);

	Vector2i trackedImageSize = trackingController->GetTrackedImageSize(imgSize_rgb, imgSize_d);

	surfelRenderState_live = new ITMSurfelRenderState(trackedImageSize, settings->surfelSceneParams.supersamplingFactor);
	surfelRenderState_freeview = NULL; //will be created if needed

	trackingState = new ITMTrackingState(trackedImageSize, memoryType);
	tracker->UpdateInitialPose(trackingState);

	view = NULL; // will be allocated by the view builder
	
	if (settings->behaviourOnFailure == settings->FAILUREMODE_RELOCALISE)
		relocaliser = new FernRelocLib::Relocaliser<float>(imgSize_d, Vector2f(settings->sceneParams.viewFrustum_min, settings->sceneParams.viewFrustum_max), 0.2f, 500, 4);
	else relocaliser = NULL;

	kfRaycast = new ITMUChar4Image(imgSize_d, memoryType);

	trackingActive = true;
	fusionActive = true;
	mainProcessingActive = true;
	trackingInitialised = false;
	relocalisationCount = 0;
	framesProcessed = 0;
}

template <typename TSurfel>
ITMBasicSurfelEngine<TSurfel>::~ITMBasicSurfelEngine()
{
	delete surfelRenderState_live;
	delete surfelRenderState_freeview;

	delete surfelScene;

	delete denseSurfelMapper;
	delete trackingController;

	delete tracker;
	delete imuCalibrator;

	delete lowLevelEngine;
	delete viewBuilder;

	delete trackingState;
	if (view != NULL) delete view;

	delete surfelVisualisationEngine;

	if (relocaliser != NULL) delete relocaliser;
	delete kfRaycast;
}

template <typename TSurfel>
void ITMBasicSurfelEngine<TSurfel>::SaveSceneToMesh(const char *objFileName)
{
	// Not yet implemented for surfel scenes
}

template <typename TSurfel>
void ITMBasicSurfelEngine<TSurfel>::SaveToFile()
{
	// Not yet implemented for surfel scenes
}

template <typename TSurfel>
void ITMBasicSurfelEngine<TSurfel>::LoadFromFile()
{
	// Not yet implemented for surfel scenes
}

template <typename TSurfel>
void ITMBasicSurfelEngine<TSurfel>::resetAll()
{
	surfelScene->Reset();
	trackingState->Reset();
}

#ifdef OUTPUT_TRAJECTORY_QUATERNIONS
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

static void QuaternionFromRotationMatrix(const double *matrix, double *q) {
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
#endif

template <typename TSurfel>
ITMTrackingState::TrackingResult ITMBasicSurfelEngine<TSurfel>::ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement)
{
	// prepare image and turn it into a depth image
	if (imuMeasurement == NULL) viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter);
	else viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, imuMeasurement);

	if (!mainProcessingActive) return ITMTrackingState::TRACKING_FAILED;

	// tracking
	ORUtils::SE3Pose oldPose(*(trackingState->pose_d));
	if (trackingActive) trackingController->Track(trackingState, view);

	ITMTrackingState::TrackingResult trackerResult = ITMTrackingState::TRACKING_GOOD;
	switch (settings->behaviourOnFailure) {
	case ITMLibSettings::FAILUREMODE_RELOCALISE:
		trackerResult = trackingState->trackerResult;
		break;
	case ITMLibSettings::FAILUREMODE_STOP_INTEGRATION:
		if (trackingState->trackerResult != ITMTrackingState::TRACKING_FAILED)
			trackerResult = trackingState->trackerResult;
		else trackerResult = ITMTrackingState::TRACKING_POOR;
		break;
	default:
		break;
	}

	//relocalisation
	int addKeyframeIdx = -1;
	if (settings->behaviourOnFailure == ITMLibSettings::FAILUREMODE_RELOCALISE)
	{
		if (trackerResult == ITMTrackingState::TRACKING_GOOD && relocalisationCount > 0) relocalisationCount--;

		int NN; float distances;
		view->depth->UpdateHostFromDevice();

		//find and add keyframe, if necessary
		bool hasAddedKeyframe = relocaliser->ProcessFrame(view->depth, trackingState->pose_d, 0, 1, &NN, &distances, trackerResult == ITMTrackingState::TRACKING_GOOD && relocalisationCount == 0);

		//frame not added and tracking failed -> we need to relocalise
		if (!hasAddedKeyframe && trackerResult == ITMTrackingState::TRACKING_FAILED)
		{
			relocalisationCount = 10;

			// Reset previous rgb frame since the rgb image is likely different than the one acquired when setting the keyframe
			view->rgb_prev->Clear();

			const FernRelocLib::PoseDatabase::PoseInScene & keyframe = relocaliser->RetrievePose(NN);
			trackingState->pose_d->SetFrom(&keyframe.pose);

			trackingController->Prepare(trackingState, surfelScene, view, surfelVisualisationEngine, surfelRenderState_live);
			surfelVisualisationEngine->FindSurfaceSuper(surfelScene, trackingState->pose_d, &view->calib.intrinsics_d, USR_RENDER, surfelRenderState_live);
			trackingController->Track(trackingState, view);

			trackerResult = trackingState->trackerResult;
		}
	}

	bool didFusion = false;
	if ((trackerResult == ITMTrackingState::TRACKING_GOOD || !trackingInitialised) && (fusionActive) && (relocalisationCount == 0)) {
		// fusion
		denseSurfelMapper->ProcessFrame(view, trackingState, surfelScene, surfelRenderState_live);
		didFusion = true;
		if (framesProcessed > 50) trackingInitialised = true;

		framesProcessed++;
	}

	if (trackerResult == ITMTrackingState::TRACKING_GOOD || trackerResult == ITMTrackingState::TRACKING_POOR)
	{
		// raycast to renderState_live for tracking and free visualisation
		trackingController->Prepare(trackingState, surfelScene, view, surfelVisualisationEngine, surfelRenderState_live);
		surfelVisualisationEngine->FindSurfaceSuper(surfelScene, trackingState->pose_d, &view->calib.intrinsics_d, USR_RENDER, surfelRenderState_live);

#if 0
		if (addKeyframeIdx >= 0)
		{
			ORUtils::MemoryBlock<Vector4u>::MemoryCopyDirection memoryCopyDirection =
				settings->deviceType == ITMLibSettings::DEVICE_CUDA ? ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CUDA : ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU;

			kfRaycast->SetFrom(renderState_live->raycastImage, memoryCopyDirection);
		}
#endif
	}
	else *trackingState->pose_d = oldPose;

#ifdef OUTPUT_TRAJECTORY_QUATERNIONS
	const ORUtils::SE3Pose *p = trackingState->pose_d;
	double t[3];
	double R[9];
	double q[4];
	for (int i = 0; i < 3; ++i) t[i] = p->GetInvM().m[3 * 4 + i];
	for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c)
		R[r * 3 + c] = p->GetM().m[c * 4 + r];
	QuaternionFromRotationMatrix(R, q);
	fprintf(stderr, "%f %f %f %f %f %f %f\n", t[0], t[1], t[2], q[1], q[2], q[3], q[0]);
#endif

		return trackerResult;
}

template <typename TSurfel>
Vector2i ITMBasicSurfelEngine<TSurfel>::GetImageSize(void) const
{
	return surfelRenderState_live->GetIndexImage()->noDims;
}

template <typename TSurfel>
typename ITMSurfelVisualisationEngine<TSurfel>::RenderImageType
ITMBasicSurfelEngine<TSurfel>::ToSurfelImageType(GetImageType getImageType)
{
	switch (getImageType)
	{
		case InfiniTAM_IMAGE_COLOUR_FROM_VOLUME:
		case InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
			return ITMSurfelVisualisationEngine<TSurfel>::RENDER_COLOUR;
		case InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
		case InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
			return ITMSurfelVisualisationEngine<TSurfel>::RENDER_NORMAL;
		case InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE:
		case InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE:
			return ITMSurfelVisualisationEngine<TSurfel>::RENDER_CONFIDENCE;
		default:
			return ITMSurfelVisualisationEngine<TSurfel>::RENDER_LAMBERTIAN;
	}
}

template <typename TSurfel>
void ITMBasicSurfelEngine<TSurfel>::GetImage(ITMUChar4Image *out, GetImageType getImageType, ORUtils::SE3Pose *pose, ITMIntrinsics *intrinsics)
{
	if (view == NULL) return;

	out->Clear();

	switch (getImageType)
	{
	case ITMBasicSurfelEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
		out->ChangeDims(view->rgb->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) 
			out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	case ITMBasicSurfelEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
		out->ChangeDims(view->depth->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depth->UpdateHostFromDevice();
		IITMVisualisationEngine::DepthToUchar4(out, view->depth);
		break;
	case ITMBasicSurfelEngine::InfiniTAM_IMAGE_SCENERAYCAST:
	case ITMBasicSurfelEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME:
	case ITMBasicSurfelEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
	case ITMBasicSurfelEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE:
		{
			const bool useRadii = true;
			surfelVisualisationEngine->FindSurface(surfelScene, trackingState->pose_d, &view->calib.intrinsics_d, useRadii, USR_DONOTRENDER, surfelRenderState_live);
			surfelVisualisationEngine->RenderImage(surfelScene, trackingState->pose_d, surfelRenderState_live, out, ToSurfelImageType(getImageType));
			out->UpdateHostFromDevice();
			break;
		}
	case ITMBasicSurfelEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED:
	case ITMBasicSurfelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
	case ITMBasicSurfelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
	case ITMBasicSurfelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE:
	{
		if (!surfelRenderState_freeview) surfelRenderState_freeview = new ITMSurfelRenderState(view->depth->noDims, surfelScene->GetParams().supersamplingFactor);
		const bool useRadii = true;
		surfelVisualisationEngine->FindSurface(surfelScene, pose, intrinsics, useRadii, USR_DONOTRENDER, surfelRenderState_freeview);
		surfelVisualisationEngine->RenderImage(surfelScene, pose, surfelRenderState_freeview, out, ToSurfelImageType(getImageType));
		out->UpdateHostFromDevice();
		break;
	}
	case ITMMainEngine::InfiniTAM_IMAGE_UNKNOWN:
		break;
	}
}

template <typename TSurfel>
void ITMBasicSurfelEngine<TSurfel>::turnOnTracking() { trackingActive = true; }

template <typename TSurfel>
void ITMBasicSurfelEngine<TSurfel>::turnOffTracking() { trackingActive = false; }

template <typename TSurfel>
void ITMBasicSurfelEngine<TSurfel>::turnOnIntegration() { fusionActive = true; }

template <typename TSurfel>
void ITMBasicSurfelEngine<TSurfel>::turnOffIntegration() { fusionActive = false; }

template <typename TSurfel>
void ITMBasicSurfelEngine<TSurfel>::turnOnMainProcessing() { mainProcessingActive = true; }

template <typename TSurfel>
void ITMBasicSurfelEngine<TSurfel>::turnOffMainProcessing() { mainProcessingActive = false; }
