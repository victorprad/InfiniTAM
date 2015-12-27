// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMLibDefines.h"
#include "../Meshing/CUDA/ITMMeshingEngine_CUDA.cu"
#include "../Reconstruction/CUDA/ITMSceneReconstructionEngine_CUDA.cu"
#include "../Swapping/CUDA/ITMSwappingEngine_CUDA.cu"
#include "../Trackers/CUDA/ITMRenTracker_CUDA.cu"
#include "../Visualisation/CUDA/ITMVisualisationEngine_CUDA.cu"

namespace ITMLib
{

template class ITMMeshingEngine_CUDA<ITMVoxel, ITMVoxelIndex>;
template class ITMRenTracker_CUDA<ITMVoxel, ITMVoxelIndex>;
template class ITMSceneReconstructionEngine_CUDA<ITMVoxel, ITMVoxelIndex>;
template class ITMSwappingEngine_CUDA<ITMVoxel, ITMVoxelIndex>;
template class ITMVisualisationEngine_CUDA<ITMVoxel, ITMVoxelIndex>;

}
