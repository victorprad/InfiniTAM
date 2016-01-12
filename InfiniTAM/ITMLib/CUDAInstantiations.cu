// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMLibDefines.h"
#include "Engines/Meshing/CUDA/ITMMeshingEngine_CUDA.tcu"
#include "Engines/Reconstruction/CUDA/ITMSceneReconstructionEngine_CUDA.tcu"
#include "Engines/Reconstruction/CUDA/ITMSurfelSceneReconstructionEngine_CUDA.tcu"
#include "Engines/Swapping/CUDA/ITMSwappingEngine_CUDA.tcu"
#include "Engines/Visualisation/CUDA/ITMSurfelVisualisationEngine_CUDA.tcu"
#include "Engines/Visualisation/CUDA/ITMVisualisationEngine_CUDA.tcu"
#include "Trackers/CUDA/ITMRenTracker_CUDA.tcu"

namespace ITMLib
{

template class ITMMeshingEngine_CUDA<ITMVoxel, ITMVoxelIndex>;
template class ITMRenTracker_CUDA<ITMVoxel, ITMVoxelIndex>;
template class ITMSceneReconstructionEngine_CUDA<ITMVoxel, ITMVoxelIndex>;
template class ITMSurfelSceneReconstructionEngine_CUDA<ITMSurfel>;
template class ITMSurfelSceneReconstructionEngine_CUDA<ITMSurfel_rgb>;
template class ITMSurfelVisualisationEngine_CUDA<ITMSurfel>;
template class ITMSurfelVisualisationEngine_CUDA<ITMSurfel_rgb>;
template class ITMSwappingEngine_CUDA<ITMVoxel, ITMVoxelIndex>;
template class ITMVisualisationEngine_CUDA<ITMVoxel, ITMVoxelIndex>;

}
