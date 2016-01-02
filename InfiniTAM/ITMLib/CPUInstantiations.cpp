// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMLibDefines.h"
#include "Core/ITMBasicEngine.tpp"
#include "Core/ITMDenseMapper.tpp"
#include "Core/ITMDenseSurfelMapper.cpp"
#include "Core/ITMLocalSceneManager.tpp"
#include "Core/ITMMultiEngine.tpp"
#include "Meshing/CPU/ITMMeshingEngine_CPU.tpp"
#include "Reconstruction/ITMSurfelSceneReconstructionEngineFactory.cpp"
#include "Reconstruction/CPU/ITMSceneReconstructionEngine_CPU.tpp"
#include "Reconstruction/CPU/ITMSurfelSceneReconstructionEngine_CPU.cpp"
#include "Reconstruction/Interface/ITMSurfelSceneReconstructionEngine.cpp"
#include "Swapping/CPU/ITMSwappingEngine_CPU.tpp"
#include "Trackers/ITMTrackerFactory.h"
#include "Trackers/CPU/ITMRenTracker_CPU.tpp"
#include "Trackers/Interface/ITMRenTracker.tpp"
#include "Visualisation/ITMSurfelVisualisationEngineFactory.cpp"
#include "Visualisation/CPU/ITMSurfelVisualisationEngine_CPU.cpp"
#include "Visualisation/CPU/ITMVisualisationEngine_CPU.tpp"
#include "Visualisation/Interface/ITMSurfelVisualisationEngine.cpp"
#include "Visualisation/Interface/ITMVisualisationEngine.h"

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
