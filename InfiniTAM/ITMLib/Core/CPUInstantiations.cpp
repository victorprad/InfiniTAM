// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMBasicEngine.cpp"
#include "ITMDenseMapper.cpp"
#include "ITMLibDefines.h"
#include "../Meshing/CPU/ITMMeshingEngine_CPU.cpp"
#include "../Reconstruction/CPU/ITMSceneReconstructionEngine_CPU.cpp"
#include "../Swapping/CPU/ITMSwappingEngine_CPU.cpp"
#include "../Trackers/ITMTrackerFactory.h"
#include "../Trackers/CPU/ITMRenTracker_CPU.cpp"
#include "../Trackers/Interface/ITMRenTracker.cpp"
#include "../Visualisation/CPU/ITMVisualisationEngine_CPU.cpp"
#include "../Visualisation/Interface/ITMVisualisationEngine.h"

namespace ITMLib
{

template class ITMBasicEngine<ITMVoxel, ITMVoxelIndex>;
template class ITMDenseMapper<ITMVoxel, ITMVoxelIndex>;
template class ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>;
template class ITMRenTracker<ITMVoxel, ITMVoxelIndex>;
template class ITMRenTracker_CPU<ITMVoxel, ITMVoxelIndex>;
template class ITMSceneReconstructionEngine_CPU<ITMVoxel, ITMVoxelIndex>;
template class ITMSwappingEngine_CPU<ITMVoxel, ITMVoxelIndex>;
template class ITMTrackerFactory<ITMVoxel, ITMVoxelIndex>;
template class ITMVisualisationEngine<ITMVoxel, ITMVoxelIndex>;
template class ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>;

}
