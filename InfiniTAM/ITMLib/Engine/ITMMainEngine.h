// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../ITMLib.h"
#include "../Utils/ITMLibSettings.h"

/** \mainpage
    This is the API reference documentation for InfiniTAM. For a general
    overview additional documentation can be found in the included Technical
    Report.

    For use of ITMLib in your own project, the class
    @ref ITMLib::Engine::ITMMainEngine should be the main interface and entry
    point to the library.
*/

namespace ITMLib
{
	namespace Engine
	{
		/** \brief
		    Main engine, that instantiates all the other engines and
		    provides a simplified interface to them.

		    This class is the main entry point to the ITMLib library
		    and basically performs the whole KinectFusion algorithm.
		    It stores the latest image internally, as well as the 3D
		    world model and additionally it keeps track of the camera
		    pose.

		    The intended use is as follows:
		    -# Create an ITMMainEngine specifying the internal settings,
		       camera parameters and image sizes
		    -# Get the pointer to the internally stored images with
		       @ref GetView() and write new image information to that
		       memory
		    -# Call the method @ref ProcessFrame() to track the camera
		       and integrate the new information into the world model
		    -# Optionally access the rendered reconstruction or another
		       image for visualisation using @ref GetImage()
		    -# Iterate the above three steps for each image in the
		       sequence

		    To access the internal information, look at the member
		    variables @ref trackingState and @ref scene.
		*/
		class ITMMainEngine
		{
		private:
			const ITMLibSettings *settings;

			bool fusionActive, mainProcessingActive;

			ITMLowLevelEngine *lowLevelEngine;
			IITMVisualisationEngine *visualisationEngine;

			ITMMeshingEngine<ITMVoxel, ITMVoxelIndex> *meshingEngine;
			ITMMesh *mesh;

			ITMViewBuilder *viewBuilder;		
			ITMDenseMapper<ITMVoxel,ITMVoxelIndex> *denseMapper;
			ITMTrackingController *trackingController;

			ITMTracker *tracker;
			ITMIMUCalibrator *imuCalibrator;

			ITMView *view;
			ITMTrackingState *trackingState;

			ITMScene<ITMVoxel, ITMVoxelIndex> *scene;
			ITMRenderState *renderState_live;
			ITMRenderState *renderState_freeview;

		public:
			enum GetImageType
			{
				InfiniTAM_IMAGE_ORIGINAL_RGB,
				InfiniTAM_IMAGE_ORIGINAL_DEPTH,
				InfiniTAM_IMAGE_SCENERAYCAST,
				InfiniTAM_IMAGE_FREECAMERA_SHADED,
				InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME,
				InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL,
				InfiniTAM_IMAGE_UNKNOWN
			};

			/// Gives access to the current input frame
			ITMView* GetView() { return view; }

			/// Gives access to the current camera pose and additional tracking information
			ITMTrackingState* GetTrackingState(void) { return trackingState; }

			/// Gives access to the internal world representation
			ITMScene<ITMVoxel, ITMVoxelIndex>* GetScene(void) { return scene; }

			/// Process a frame with rgb and depth images and optionally a corresponding imu measurement
			void ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement = NULL);

			// Gives access to the data structure used internally to store any created meshes
			ITMMesh* GetMesh(void) { return mesh; }

			/// Update the internally stored mesh data structure and return a pointer to it
			ITMMesh* UpdateMesh(void);

			/// Extracts a mesh from the current scene and saves it to the obj file specified by the file name
			void SaveSceneToMesh(const char *objFileName);

			/// Get a result image as output
			Vector2i GetImageSize(void) const;

			void GetImage(ITMUChar4Image *out, GetImageType getImageType, ITMPose *pose = NULL, ITMIntrinsics *intrinsics = NULL);

			/// switch for turning intergration on/off
			void turnOnIntegration();
			void turnOffIntegration();

			/// switch for turning main processing on/off
			void turnOnMainProcessing();
			void turnOffMainProcessing();

			/** \brief Constructor
			    Ommitting a separate image size for the depth images
			    will assume same resolution as for the RGB images.
			*/
			ITMMainEngine(const ITMLibSettings *settings, const ITMRGBDCalib *calib, Vector2i imgSize_rgb, Vector2i imgSize_d = Vector2i(-1,-1));
			~ITMMainEngine();
		};
	}
}

