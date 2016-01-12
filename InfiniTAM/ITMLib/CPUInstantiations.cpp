// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMLibDefines.h"
#include "Core/ITMBasicEngine.tpp"
#include "Core/ITMDenseMapper.tpp"
#include "Core/ITMDenseSurfelMapper.tpp"
#include "Core/ITMLocalSceneManager.tpp"
#include "Core/ITMMultiEngine.tpp"
#include "Engines/Meshing/CPU/ITMMeshingEngine_CPU.tpp"
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
#include "Trackers/CPU/ITMRenTracker_CPU.tpp"
#include "Trackers/Interface/ITMRenTracker.tpp"

namespace ITMLib
{

template class ITMBasicEngine<ITMVoxel, ITMVoxelIndex>;
template class ITMDenseMapper<ITMVoxel, ITMVoxelIndex>;
template class ITMDenseSurfelMapper<ITMSurfel>;
template class ITMDenseSurfelMapper<ITMSurfel_rgb>;
template class ITMLocalSceneManager_instance<ITMVoxel, ITMVoxelIndex>;
template class ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>;
template class ITMMultiEngine<ITMVoxel, ITMVoxelIndex>;
template class ITMRenTracker<ITMVoxel, ITMVoxelIndex>;
template class ITMRenTracker_CPU<ITMVoxel, ITMVoxelIndex>;
template class ITMSceneReconstructionEngine_CPU<ITMVoxel, ITMVoxelIndex>;
template class ITMSurfelSceneReconstructionEngine<ITMSurfel>;
template class ITMSurfelSceneReconstructionEngine<ITMSurfel_rgb>;
template class ITMSurfelSceneReconstructionEngine_CPU<ITMSurfel>;
template class ITMSurfelSceneReconstructionEngine_CPU<ITMSurfel_rgb>;
template struct ITMSurfelSceneReconstructionEngineFactory<ITMSurfel>;
template struct ITMSurfelSceneReconstructionEngineFactory<ITMSurfel_rgb>;
template class ITMSurfelVisualisationEngine<ITMSurfel>;
template class ITMSurfelVisualisationEngine<ITMSurfel_rgb>;
template class ITMSurfelVisualisationEngine_CPU<ITMSurfel>;
template class ITMSurfelVisualisationEngine_CPU<ITMSurfel_rgb>;
template struct ITMSurfelVisualisationEngineFactory<ITMSurfel>;
template struct ITMSurfelVisualisationEngineFactory<ITMSurfel_rgb>;
template class ITMSwappingEngine_CPU<ITMVoxel, ITMVoxelIndex>;
template class ITMTrackerFactory<ITMVoxel, ITMVoxelIndex>;
template class ITMVisualisationEngine<ITMVoxel, ITMVoxelIndex>;
template class ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>;

}
