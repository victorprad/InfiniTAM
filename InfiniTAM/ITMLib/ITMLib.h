// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#ifndef __InfiniTAM_LIB__
#define __InfiniTAM_LIB__

#include "Utils/ITMLibDefines.h"

#include "Objects/ITMScene.h"
#include "Objects/ITMView.h"

#include "LowLevel/Interface/ITMLowLevelEngine.h"
#include "LowLevel/CPU/ITMLowLevelEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "LowLevel/CUDA/ITMLowLevelEngine_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "LowLevel/Metal/ITMLowLevelEngine_Metal.h"
#endif

#include "Trackers/Interface/ITMDepthTracker.h"
#include "Trackers/CPU/ITMDepthTracker_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Trackers/CUDA/ITMDepthTracker_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Trackers/Metal/ITMDepthTracker_Metal.h"
#endif

#include "Trackers/Interface/ITMWeightedICPTracker.h"
#include "Trackers/CPU/ITMWeightedICPTracker_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Trackers/CUDA/ITMWeightedICPTracker_CUDA.h"
#endif

#include "Reconstruction/Interface/ITMSceneReconstructionEngine.h"
#include "Reconstruction/CPU/ITMSceneReconstructionEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Reconstruction/CUDA/ITMSceneReconstructionEngine_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Reconstruction/Metal/ITMSceneReconstructionEngine_Metal.h"
#endif

#include "Visualisation/Interface/ITMVisualisationEngine.h"
#include "Visualisation/CPU/ITMVisualisationEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Visualisation/CUDA/ITMVisualisationEngine_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Visualisation/Metal/ITMVisualisationEngine_Metal.h"
#endif

#include "Trackers/Interface/ITMColorTracker.h"
#include "Trackers/CPU/ITMColorTracker_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Trackers/CUDA/ITMColorTracker_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Trackers/Metal/ITMColorTracker_Metal.h"
#endif

#include "Swapping/Interface/ITMSwappingEngine.h"
#include "Swapping/CPU/ITMSwappingEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Swapping/CUDA/ITMSwappingEngine_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Swapping/Metal/ITMSwappingEngine_Metal.h"
#endif

#include "Trackers/Interface/ITMRenTracker.h"
#include "Trackers/CPU/ITMRenTracker_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Trackers/CUDA/ITMRenTracker_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Trackers/Metal/ITMRenTracker_Metal.h"
#endif

#include "Trackers/Interface/ITMIMUTracker.h"
#include "Trackers/Interface/ITMCompositeTracker.h"
#include "Engine/ITMTrackingController.h"

#include "ViewBuilding/Interface/ITMViewBuilder.h"
#include "ViewBuilding/CPU/ITMViewBuilder_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "ViewBuilding/CUDA/ITMViewBuilder_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "ViewBuilding/Metal/ITMViewBuilder_Metal.h"
#endif

#include "Meshing/Interface/ITMMeshingEngine.h"
#include "Meshing/CPU/ITMMeshingEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Meshing/CUDA/ITMMeshingEngine_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Meshing/CPU/ITMMeshingEngine_CPU.h"
#endif

#include "Engine/ITMDenseMapper.h"
#include "Engine/ITMMainEngine.h"

#endif
