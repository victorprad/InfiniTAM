// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMLibDefines.h"
#include "Engines/Reconstruction/Metal/ITMSceneReconstructionEngine_Metal.mm"
#include "Engines/Visualisation/Metal/ITMVisualisationEngine_Metal.mm"

namespace ITMLib
{
    template class ITMLib::ITMSceneReconstructionEngine_Metal<ITMVoxel, ITMVoxelIndex>;
    template class ITMLib::ITMVisualisationEngine_Metal<ITMVoxel, ITMVoxelIndex>;
}
