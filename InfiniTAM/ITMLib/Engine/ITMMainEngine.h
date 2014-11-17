// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

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
			ITMLibSettings *settings;

			bool hasStartedObjectReconstruction;

			ITMSceneReconstructionEngine<ITMVoxel,ITMVoxelIndex> *sceneRecoEngine;
			ITMTracker *trackerPrimary, *trackerSecondary;
			ITMLowLevelEngine *lowLevelEngine;
			ITMSwappingEngine<ITMVoxel,ITMVoxelIndex> *swappingEngine;
			ITMVisualisationEngine<ITMVoxel,ITMVoxelIndex> *visualisationEngine;
			ITMVisualisationState *visualisationState;
		public:
			enum GetImageType
			{
				InfiniTAM_IMAGE_ORIGINAL_RGB,
				InfiniTAM_IMAGE_ORIGINAL_DEPTH,
				InfiniTAM_IMAGE_SCENERAYCAST,
				InfiniTAM_IMAGE_SCENERAYCAST_FREECAMERA
			};

			/// Pointer to the current model of the 3D scene
			ITMScene<ITMVoxel,ITMVoxelIndex> *scene;
			/// Pointer for storing the current input frame
			ITMView *view;
			/// Pointer to the current camera pose and additional tracking information
			ITMTrackingState *trackingState;

			/// Gives access to the current input frame
			ITMView* GetView() { return view; }

			/// Process the frame accessed with @ref GetView()
			void ProcessFrame(void);

			/// Get a result image as output
			void GetImage(ITMUChar4Image *out, GetImageType getImageType, bool useColour, ITMPose *pose = NULL, ITMIntrinsics *intrinsics = NULL);

			void SaveAll();

			/** \brief Constructor
			    Ommitting a separate image size for the depth images
			    will assume same resolution as for the RGB images.
			*/
			ITMMainEngine(const ITMLibSettings *settings, const ITMRGBDCalib *calib, Vector2i imgSize_rgb, Vector2i imgSize_d = Vector2i(-1,-1));
			~ITMMainEngine();
		};
	}
}

