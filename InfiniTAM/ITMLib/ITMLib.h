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

#include "Engine/ITMDepthTracker.h"
#include "Engine/DeviceSpecific/CPU/ITMDepthTracker_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Engine/DeviceSpecific/CUDA/ITMDepthTracker_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Engine/DeviceSpecific/Metal/ITMDepthTracker_Metal.h"
#endif

#include "Engine/ITMWeightedICPTracker.h"
#include "Engine/DeviceSpecific/CPU/ITMWeightedICPTracker_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Engine/DeviceSpecific/CUDA/ITMWeightedICPTracker_CUDA.h"
#endif

#include "Engine/ITMSceneReconstructionEngine.h"
#include "Engine/DeviceSpecific/CPU/ITMSceneReconstructionEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Engine/DeviceSpecific/CUDA/ITMSceneReconstructionEngine_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Engine/DeviceSpecific/Metal/ITMSceneReconstructionEngine_Metal.h"
#endif

#include "Engine/ITMVisualisationEngine.h"
#include "Engine/DeviceSpecific/CPU/ITMVisualisationEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Engine/DeviceSpecific/CUDA/ITMVisualisationEngine_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Engine/DeviceSpecific/Metal/ITMVisualisationEngine_Metal.h"
#endif

#include "Engine/ITMColorTracker.h"
#include "Engine/DeviceSpecific/CPU/ITMColorTracker_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Engine/DeviceSpecific/CUDA/ITMColorTracker_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Engine/DeviceSpecific/Metal/ITMColorTracker_Metal.h"
#endif

#include "Engine/ITMSwappingEngine.h"
#include "Engine/DeviceSpecific/CPU/ITMSwappingEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Engine/DeviceSpecific/CUDA/ITMSwappingEngine_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Engine/DeviceSpecific/Metal/ITMSwappingEngine_Metal.h"
#endif

#include "Engine/ITMRenTracker.h"
#include "Engine/DeviceSpecific/CPU/ITMRenTracker_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Engine/DeviceSpecific/CUDA/ITMRenTracker_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Engine/DeviceSpecific/Metal/ITMRenTracker_Metal.h"
#endif

#include "Engine/ITMIMUTracker.h"
#include "Engine/ITMCompositeTracker.h"
#include "Engine/ITMTrackingController.h"

#include "Engine/ITMViewBuilder.h"
#include "Engine/DeviceSpecific/CPU/ITMViewBuilder_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Engine/DeviceSpecific/CUDA/ITMViewBuilder_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Engine/DeviceSpecific/Metal/ITMViewBuilder_Metal.h"
#endif

#include "Engine/ITMMeshingEngine.h"
#include "Engine/DeviceSpecific/CPU/ITMMeshingEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Engine/DeviceSpecific/CUDA/ITMMeshingEngine_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Engine/DeviceSpecific/CPU/ITMMeshingEngine_CPU.h"
#endif

#include "Engine/ITMDenseMapper.h"
#include "Engine/ITMMainEngine.h"

using namespace ITMLib::Objects;
using namespace ITMLib::Engine;

#endif
