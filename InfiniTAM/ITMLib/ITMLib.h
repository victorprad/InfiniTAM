// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#ifndef __InfiniTAM_LIB__
#define __InfiniTAM_LIB__

#include "Utils/ITMLibDefines.h"

#include "Objects/ITMScene.h"
#include "Objects/ITMView.h"
#include "Objects/ITMTrackingState.h"
#include "Objects/ITMVisualisationState.h"

#include "Engine/ITMLowLevelEngine.h"
#include "Engine/DeviceSpecific/CPU/ITMLowLevelEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Engine/DeviceSpecific/CUDA/ITMLowLevelEngine_CUDA.h"
#endif

#include "Engine/ITMDepthTracker.h"
#include "Engine/DeviceSpecific/CPU/ITMDepthTracker_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Engine/DeviceSpecific/CUDA/ITMDepthTracker_CUDA.h"
#endif

#include "Engine/ITMSceneReconstructionEngine.h"
#include "Engine/DeviceSpecific/CPU/ITMSceneReconstructionEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Engine/DeviceSpecific/CUDA/ITMSceneReconstructionEngine_CUDA.h"
#endif

#include "Engine/ITMVisualisationEngine.h"
#include "Engine/DeviceSpecific/CPU/ITMVisualisationEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Engine/DeviceSpecific/CUDA/ITMVisualisationEngine_CUDA.h"
#endif

#include "Engine/ITMColorTracker.h"
#include "Engine/DeviceSpecific/CPU/ITMColorTracker_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Engine/DeviceSpecific/CUDA/ITMColorTracker_CUDA.h"
#endif

#include "Engine/ITMSwappingEngine.h"
#include "Engine/DeviceSpecific/CPU/ITMSwappingEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Engine/DeviceSpecific/CUDA/ITMSwappingEngine_CUDA.h"
#endif

#include "Engine/ITMRenTracker.h"
#include "Engine/DeviceSpecific/CPU/ITMRenTracker_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "Engine/DeviceSpecific/CUDA/ITMRenTracker_CUDA.h"
#endif

#include "Engine/ITMVisualisationEngine.h"
#include "Engine/ITMMainEngine.h"

using namespace ITMLib::Objects;
using namespace ITMLib::Engine;

#endif
