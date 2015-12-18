// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#ifndef __InfiniTAM_LIB__
#define __InfiniTAM_LIB__

#include "Utils/ITMLibDefines.h"

#include "Objects/ITMScene.h"
#include "Objects/ITMView.h"

#include "Engine/ITMLowLevelEngine.h"
#include "Engine/DeviceSpecific/CPU/ITMLowLevelEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Engine/DeviceSpecific/CUDA/ITMLowLevelEngine_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Engine/DeviceSpecific/Metal/ITMLowLevelEngine_Metal.h"
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

#include "Engine/ITMSceneReconstructionEngine.h"
#include "Engine/DeviceSpecific/CPU/ITMSceneReconstructionEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Engine/DeviceSpecific/CUDA/ITMSceneReconstructionEngine_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Engine/DeviceSpecific/Metal/ITMSceneReconstructionEngine_Metal.h"
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

#include "Engine/ITMSwappingEngine.h"
#include "Engine/DeviceSpecific/CPU/ITMSwappingEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Engine/DeviceSpecific/CUDA/ITMSwappingEngine_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Engine/DeviceSpecific/Metal/ITMSwappingEngine_Metal.h"
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

#include "Engine/ITMViewBuilder.h"
#include "Engine/DeviceSpecific/CPU/ITMViewBuilder_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Engine/DeviceSpecific/CUDA/ITMViewBuilder_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Engine/DeviceSpecific/Metal/ITMViewBuilder_Metal.h"
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
