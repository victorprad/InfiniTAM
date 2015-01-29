// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"
#include "../Utils/ITMLibSettings.h"

#include "../Objects/ITMScene.h"
#include "../Objects/ITMTrackingState.h"
#include "../Objects/ITMRenderState.h"

#include "ITMSceneReconstructionEngine.h"
#include "ITMVisualisationEngine.h"
#include "ITMSwappingEngine.h"
#include "ITMVoxelBlockOpEngine.h"

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
			ITMVoxelBlockOpEngine<TVoxel,TIndex> *voxelBlockOpEngine;
			ITMScene<TVoxel,TIndex> *scene;

		public:
			/// Pointer to information used by the raycaster
			ITMRenderState *renderState_live;

			/// Process a single frame
			void ProcessFrame(const ITMView *view, const ITMTrackingState *trackingState);

			/// Get pointer to the current model of the 3D scene
			const ITMScene<TVoxel,TIndex> *getScene() const { return scene; } 

			/** \brief Constructor
			    Ommitting a separate image size for the depth images
			    will assume same resolution as for the RGB images.
			*/
			ITMDenseMapper(const ITMLibSettings *settings, ITMScene<TVoxel, TIndex>* scene, ITMRenderState *renderState_live);
			~ITMDenseMapper();
		};
	}
}

