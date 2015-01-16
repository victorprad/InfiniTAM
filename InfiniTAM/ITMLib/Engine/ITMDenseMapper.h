// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"
#include "../Utils/ITMLibSettings.h"

#include "../Objects/ITMScene.h"
#include "../Objects/ITMTrackingState.h"
#include "../Objects/ITMRenderState.h"

#include "../Engine/ITMSceneReconstructionEngine.h"
#include "../Engine/ITMVisualisationEngine.h"
#include "../Engine/ITMSwappingEngine.h"

namespace ITMLib
{
	namespace Engine
	{
		/** \brief
		*/
		template<class TVoxel, class TIndex>
		class ITMDenseMapper
		{
		private:
			const ITMLibSettings *settings;

			ITMSceneReconstructionEngine<TVoxel,TIndex> *sceneRecoEngine;
			ITMSwappingEngine<TVoxel,TIndex> *swappingEngine;
			ITMVisualisationEngine<TVoxel,TIndex> *visualisationEngine;
			ITMScene<TVoxel,TIndex> *scene;

			ITMRenderState *renderState_freeview;

		public:
			/// Pointer to information used by the raycaster
			ITMRenderState *renderState_live;

			/// Process a single frame
			void ProcessFrame(const ITMView *view, const ITMTrackingState *trackingState);

			/// Get pointer to the current model of the 3D scene
			const ITMScene<TVoxel,TIndex> *getScene() const
			{ return scene; } 

//			const ITMVisualisationEngine<TVoxel,TIndex> *getVisualisationEngine(void) const { return visualisationEngine; }

			void GetICPMaps(const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMView *view, ITMTrackingState *trackingState);
			void GetPointCloud(const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMView *view, ITMTrackingState *trackingState, bool skipPoints);
			void GetRendering(const ITMPose *pose, const ITMIntrinsics *intrinsics, bool useColour, ITMUChar4Image *out);

			/** \brief Constructor
			    Ommitting a separate image size for the depth images
			    will assume same resolution as for the RGB images.
			*/
			ITMDenseMapper(const ITMLibSettings *settings, /*const ITMRGBDCalib *calib,*/ Vector2i imgSize_rgb, Vector2i imgSize_d = Vector2i(-1,-1));
			~ITMDenseMapper();
		};
	}
}

