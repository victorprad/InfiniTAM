// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMLibDefines.h"
#include "Core/ITMBasicEngine.tpp"
#include "Core/ITMDenseMapper.tpp"
#include "Core/ITMLocalSceneManager.tpp"
#include "Core/ITMMultiEngine.tpp"
#include "Meshing/CPU/ITMMeshingEngine_CPU.tpp"
#include "Reconstruction/CPU/ITMSceneReconstructionEngine_CPU.tpp"
#include "Swapping/CPU/ITMSwappingEngine_CPU.tpp"
#include "Trackers/ITMTrackerFactory.h"
#include "Trackers/CPU/ITMRenTracker_CPU.tpp"
#include "Trackers/Interface/ITMRenTracker.tpp"
#include "Visualisation/CPU/ITMVisualisationEngine_CPU.tpp"
#include "Visualisation/Interface/ITMVisualisationEngine.h"

namespace ITMLib
{

template class ITMBasicEngine<ITMVoxel, ITMVoxelIndex>;
template class ITMDenseMapper<ITMVoxel, ITMVoxelIndex>;
template class ITMLocalSceneManager_instance<ITMVoxel, ITMVoxelIndex>;
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
