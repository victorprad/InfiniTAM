// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "Engine/ITMDenseMapper.cpp"
#include "Meshing/CPU/ITMMeshingEngine_CPU.cpp"
#include "Reconstruction/CPU/ITMSceneReconstructionEngine_CPU.cpp"
#include "Swapping/CPU/ITMSwappingEngine_CPU.cpp"
#include "Trackers/CPU/ITMRenTracker_CPU.cpp"
#include "Utils/ITMLibDefines.h"
#include "Visualisation/CPU/ITMVisualisationEngine_CPU.cpp"

namespace ITMLib
{

template class ITMDenseMapper<ITMVoxel, ITMVoxelIndex>;
template class ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>;
template class ITMRenTracker_CPU<ITMVoxel, ITMVoxelIndex>;
template class ITMSceneReconstructionEngine_CPU<ITMVoxel, ITMVoxelIndex>;
template class ITMSwappingEngine_CPU<ITMVoxel, ITMVoxelIndex>;
template class ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>;

}
