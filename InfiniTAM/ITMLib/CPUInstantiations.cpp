// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMLibDefines.h"
#include "Core/ITMBasicEngine.tpp"
#include "Core/ITMBasicSurfelEngine.tpp"
#include "Core/ITMMultiEngine.tpp"
#include "Core/ITMDenseMapper.tpp"
#include "Core/ITMDenseSurfelMapper.tpp"
#include "Engines/Meshing/CPU/ITMMeshingEngine_CPU.tpp"
#include "Engines/Meshing/CPU/ITMMultiMeshingEngine_CPU.tpp"
#include "Engines/MultiScene/ITMMapGraphManager.tpp"
#include "Engines/Visualisation/CPU/ITMMultiVisualisationEngine_CPU.tpp"
#include "Engines/Reconstruction/ITMSurfelSceneReconstructionEngineFactory.tpp"
#include "Engines/Reconstruction/CPU/ITMSceneReconstructionEngine_CPU.tpp"
#include "Engines/Reconstruction/CPU/ITMSurfelSceneReconstructionEngine_CPU.tpp"
#include "Engines/Reconstruction/Interface/ITMSurfelSceneReconstructionEngine.tpp"
#include "Engines/Swapping/CPU/ITMSwappingEngine_CPU.tpp"
#include "Engines/Visualisation/ITMSurfelVisualisationEngineFactory.tpp"
#include "Engines/Visualisation/CPU/ITMSurfelVisualisationEngine_CPU.tpp"
#include "Engines/Visualisation/CPU/ITMVisualisationEngine_CPU.tpp"
#include "Engines/Visualisation/Interface/ITMSurfelVisualisationEngine.tpp"
#include "Engines/Visualisation/Interface/ITMVisualisationEngine.h"
#include "Trackers/ITMTrackerFactory.h"

namespace ITMLib
{
	template class ITMBasicEngine<ITMVoxel, ITMVoxelIndex>;
	template class ITMBasicSurfelEngine<ITMSurfelT>;
  template class ITMMultiEngine<ITMVoxel, ITMVoxelIndex>;
	template class ITMDenseMapper<ITMVoxel, ITMVoxelIndex>;
	template class ITMVoxelMapGraphManager<ITMVoxel, ITMVoxelIndex>;
	template class ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>;
	template class ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>;
	template class ITMMultiMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>;
	template class ITMSwappingEngine_CPU<ITMVoxel, ITMVoxelIndex>;
	template class ITMSceneReconstructionEngine_CPU<ITMVoxel, ITMVoxelIndex>;

	template class ITMDenseSurfelMapper<ITMSurfelT>;
	template class ITMSurfelSceneReconstructionEngine<ITMSurfelT>;
	template class ITMSurfelSceneReconstructionEngine_CPU<ITMSurfelT>;
	template struct ITMSurfelSceneReconstructionEngineFactory<ITMSurfelT>;
	template class ITMSurfelVisualisationEngine<ITMSurfelT>;
	template class ITMSurfelVisualisationEngine_CPU<ITMSurfelT>;
	template struct ITMSurfelVisualisationEngineFactory<ITMSurfelT>;
}
