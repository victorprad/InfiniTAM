// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMLibDefines.h"
#include "Core/ITMBasicEngine.tpp"
#include "Core/ITMDenseMapper.tpp"
#include "Core/ITMMultiEngine.tpp"
#include "Core/MultiScene/ITMMultiSceneManager.tpp"
#include "Engines/Meshing/CPU/ITMMeshingEngine_CPU.tpp"
#include "Engines/Reconstruction/CPU/ITMSceneReconstructionEngine_CPU.tpp"
#include "Engines/Swapping/CPU/ITMSwappingEngine_CPU.tpp"
#include "Engines/Visualisation/CPU/ITMVisualisationEngine_CPU.tpp"
#include "Engines/Visualisation/Interface/ITMVisualisationEngine.h"
#include "Trackers/ITMTrackerFactory.h"
#include "Trackers/CPU/ITMRenTracker_CPU.tpp"
#include "Trackers/Interface/ITMRenTracker.tpp"

namespace ITMLib
{

template class ITMBasicEngine<ITMVoxel, ITMVoxelIndex>;
template class ITMDenseMapper<ITMVoxel, ITMVoxelIndex>;
template class ITMMultiSceneManager_instance<ITMVoxel, ITMVoxelIndex>;
template class ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>;
template class ITMMultiEngine<ITMVoxel, ITMVoxelIndex>;
template class ITMRenTracker<ITMVoxel, ITMVoxelIndex>;
template class ITMRenTracker_CPU<ITMVoxel, ITMVoxelIndex>;
template class ITMSceneReconstructionEngine_CPU<ITMVoxel, ITMVoxelIndex>;
template class ITMSwappingEngine_CPU<ITMVoxel, ITMVoxelIndex>;
template class ITMTrackerFactory<ITMVoxel, ITMVoxelIndex>;
template class ITMVisualisationEngine<ITMVoxel, ITMVoxelIndex>;
template class ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>;

}
