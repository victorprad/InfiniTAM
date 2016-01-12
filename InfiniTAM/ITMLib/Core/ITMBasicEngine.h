// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMDenseMapper.h"
#include "ITMDenseSurfelMapper.h"
#include "ITMMainEngine.h"
#include "ITMTrackingController.h"
#include "../Engines/LowLevel/Interface/ITMLowLevelEngine.h"
#include "../Engines/Meshing/Interface/ITMMeshingEngine.h"
#include "../Engines/ViewBuilding/Interface/ITMViewBuilder.h"
#include "../Engines/Visualisation/Interface/ITMSurfelVisualisationEngine.h"
#include "../Engines/Visualisation/Interface/ITMVisualisationEngine.h"
#include "../Objects/Misc/ITMIMUCalibrator.h"

namespace ITMLib
{
	template <typename TVoxel, typename TIndex>
	class ITMBasicEngine : public ITMMainEngine
	{
	private:
		typedef ITMSurfel_rgb TSurfel;

		const ITMLibSettings *settings;

		bool trackingActive, fusionActive, mainProcessingActive, trackingInitialised;

		ITMLowLevelEngine *lowLevelEngine;
		ITMSurfelVisualisationEngine<TSurfel> *surfelVisualisationEngine;
		ITMVisualisationEngine<TVoxel,TIndex> *visualisationEngine;

		ITMMeshingEngine<TVoxel,TIndex> *meshingEngine;
		ITMMesh *mesh;

		ITMViewBuilder *viewBuilder;
		ITMDenseMapper<TVoxel,TIndex> *denseMapper;
		ITMDenseSurfelMapper<TSurfel> *denseSurfelMapper;
		ITMTrackingController *trackingController;

		ITMScene<TVoxel,TIndex> *scene;
		ITMSurfelScene<TSurfel> *surfelScene;
		ITMRenderState *renderState_live;
		ITMRenderState *renderState_freeview;
		ITMSurfelRenderState *surfelRenderState_live;

		ITMTracker *tracker;
		ITMIMUCalibrator *imuCalibrator;

		/// Pointer for storing the current input frame
		ITMView *view;
		
		/// Pointer to the current camera pose and additional tracking information
		ITMTrackingState *trackingState;

	public:
		ITMView* GetView(void) { return view; }
		ITMTrackingState* GetTrackingState(void) { return trackingState; }

		/// Gives access to the internal world representation
		ITMScene<TVoxel,TIndex>* GetScene(void) { return scene; }

		void ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement = NULL);

		/// Gives access to the data structure used internally to store any created meshes
		ITMMesh* GetMesh(void) { return mesh; }

		/// Update the internally stored mesh data structure and return a pointer to it
		ITMMesh* UpdateMesh(void);

		/// Extracts a mesh from the current scene and saves it to the obj file specified by the file name
		void SaveSceneToMesh(const char *objFileName);

		/// Get a result image as output
		Vector2i GetImageSize(void) const;

		void GetImage(ITMUChar4Image *out, GetImageType getImageType, ITMPose *pose = NULL, ITMIntrinsics *intrinsics = NULL);

		/// switch for turning tracking on/off
		void turnOnTracking();
		void turnOffTracking();

		/// switch for turning integration on/off
		void turnOnIntegration();
		void turnOffIntegration();

		/// switch for turning main processing on/off
		void turnOnMainProcessing();
		void turnOffMainProcessing();

		/** \brief Constructor
		    Ommitting a separate image size for the depth images
		    will assume same resolution as for the RGB images.
		*/
		ITMBasicEngine(const ITMLibSettings *settings, const ITMRGBDCalib *calib, Vector2i imgSize_rgb, Vector2i imgSize_d = Vector2i(-1,-1));
		~ITMBasicEngine();
	};
}
