// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Engines/Reconstruction/Interface/ITMSceneReconstructionEngine.h"
#include "../Engines/Swapping/Interface/ITMSwappingEngine.h"
#include "../Utils/ITMLibSettings.h"

namespace ITMLib
{
	/** \brief
	*/
	template<class TVoxel, class TIndex>
	class ITMDenseMapper
	{
	private:
		ITMSceneReconstructionEngine<TVoxel,TIndex> *sceneRecoEngine;
		ITMSwappingEngine<TVoxel,TIndex> *swappingEngine;

		ITMLibSettings::SwappingMode swappingMode;

	public:
		void ResetScene(ITMScene<TVoxel,TIndex> *scene) const;

		/// Process a single frame
		void ProcessFrame(const ITMView *view, const ITMTrackingState *trackingState, ITMScene<TVoxel,TIndex> *scene, ITMRenderState *renderState_live, bool resetVisibleList = false);

		/// Update the visible list (this can be called to update the visible list when fusion is turned off)
		void UpdateVisibleList(const ITMView *view, const ITMTrackingState *trackingState, ITMScene<TVoxel, TIndex> *scene, ITMRenderState *renderState, bool resetVisibleList = false);

		/** \brief Constructor
		    Ommitting a separate image size for the depth images
		    will assume same resolution as for the RGB images.
		*/
		explicit ITMDenseMapper(const ITMLibSettings *settings);
		~ITMDenseMapper();
	};
}
