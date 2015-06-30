#############################
# Specify the project files #
#############################

##
SET(ITMLIB_TOPLEVEL_HEADERS
ITMLib.h
)

##
SET(ITMLIB_ENGINE_SOURCES
Engine/ITMColorTracker.cpp
Engine/ITMDenseMapper.cpp
Engine/ITMDepthTracker.cpp
Engine/ITMWeightedICPTracker.cpp
Engine/ITMIMUTracker.cpp
Engine/ITMMainEngine.cpp
Engine/ITMRenTracker.cpp
Engine/ITMTrackerFactory.cpp
Engine/ITMTrackingController.cpp
Engine/ITMVisualisationEngine.cpp
)

SET(ITMLIB_ENGINE_HEADERS
Engine/ITMColorTracker.h
Engine/ITMCompositeTracker.h
Engine/ITMDenseMapper.h
Engine/ITMDepthTracker.h
Engine/ITMWeightedICPTracker.h
Engine/ITMIMUCalibrator.h
Engine/ITMIMUTracker.h
Engine/ITMLowLevelEngine.h
Engine/ITMMainEngine.h
Engine/ITMRenTracker.h
Engine/ITMSceneReconstructionEngine.h
Engine/ITMSwappingEngine.h
Engine/ITMTracker.h
Engine/ITMTrackerFactory.h
Engine/ITMTrackingController.h
Engine/ITMViewBuilder.h
Engine/ITMVisualisationEngine.h
Engine/ITMMeshingEngine.h
)

##
set(ITMLIB_ENGINE_DEVICEAGNOSTIC_HEADERS
Engine/DeviceAgnostic/ITMColorTracker.h
Engine/DeviceAgnostic/ITMDepthTracker.h
Engine/DeviceAgnostic/ITMWeightedICPTracker.h
Engine/DeviceAgnostic/ITMLowLevelEngine.h
Engine/DeviceAgnostic/ITMPixelUtils.h
Engine/DeviceAgnostic/ITMRenTracker.h
Engine/DeviceAgnostic/ITMRepresentationAccess.h
Engine/DeviceAgnostic/ITMSceneReconstructionEngine.h
Engine/DeviceAgnostic/ITMSwappingEngine.h
Engine/DeviceAgnostic/ITMViewBuilder.h
Engine/DeviceAgnostic/ITMVisualisationEngine.h
Engine/DeviceAgnostic/ITMPixelUtils.h
Engine/DeviceAgnostic/ITMMeshingEngine.h
)

##
set(ITMLIB_ENGINE_DEVICESPECIFIC_CPU_SOURCES
Engine/DeviceSpecific/CPU/ITMColorTracker_CPU.cpp
Engine/DeviceSpecific/CPU/ITMDepthTracker_CPU.cpp
Engine/DeviceSpecific/CPU/ITMWeightedICPTracker_CPU.cpp
Engine/DeviceSpecific/CPU/ITMLowLevelEngine_CPU.cpp
Engine/DeviceSpecific/CPU/ITMRenTracker_CPU.cpp
Engine/DeviceSpecific/CPU/ITMSceneReconstructionEngine_CPU.cpp
Engine/DeviceSpecific/CPU/ITMSwappingEngine_CPU.cpp
Engine/DeviceSpecific/CPU/ITMViewBuilder_CPU.cpp
Engine/DeviceSpecific/CPU/ITMVisualisationEngine_CPU.cpp
Engine/DeviceSpecific/CPU/ITMMeshingEngine_CPU.cpp
)

set(ITMLIB_ENGINE_DEVICESPECIFIC_CPU_HEADERS
Engine/DeviceSpecific/CPU/ITMColorTracker_CPU.h
Engine/DeviceSpecific/CPU/ITMDepthTracker_CPU.h
Engine/DeviceSpecific/CPU/ITMWeightedICPTracker_CPU.h
Engine/DeviceSpecific/CPU/ITMLowLevelEngine_CPU.h
Engine/DeviceSpecific/CPU/ITMRenTracker_CPU.h
Engine/DeviceSpecific/CPU/ITMSceneReconstructionEngine_CPU.h
Engine/DeviceSpecific/CPU/ITMSwappingEngine_CPU.h
Engine/DeviceSpecific/CPU/ITMViewBuilder_CPU.h
Engine/DeviceSpecific/CPU/ITMVisualisationEngine_CPU.h
Engine/DeviceSpecific/CPU/ITMMeshingEngine_CPU.h
)

##
set(ITMLIB_ENGINE_DEVICESPECIFIC_CUDA_SOURCES
Engine/DeviceSpecific/CUDA/ITMColorTracker_CUDA.cu
Engine/DeviceSpecific/CUDA/ITMDepthTracker_CUDA.cu
Engine/DeviceSpecific/CUDA/ITMWeightedICPTracker_CUDA.cu
Engine/DeviceSpecific/CUDA/ITMLowLevelEngine_CUDA.cu
Engine/DeviceSpecific/CUDA/ITMRenTracker_CUDA.cu
Engine/DeviceSpecific/CUDA/ITMSceneReconstructionEngine_CUDA.cu
Engine/DeviceSpecific/CUDA/ITMSwappingEngine_CUDA.cu
Engine/DeviceSpecific/CUDA/ITMViewBuilder_CUDA.cu
Engine/DeviceSpecific/CUDA/ITMVisualisationEngine_CUDA.cu
Engine/DeviceSpecific/CUDA/ITMMeshingEngine_CUDA.cu
)

set(ITMLIB_ENGINE_DEVICESPECIFIC_CUDA_HEADERS
Engine/DeviceSpecific/CUDA/ITMColorTracker_CUDA.h
Engine/DeviceSpecific/CUDA/ITMCUDAUtils.h
Engine/DeviceSpecific/CUDA/ITMDepthTracker_CUDA.h
Engine/DeviceSpecific/CUDA/ITMWeightedICPTracker_CUDA.h
Engine/DeviceSpecific/CUDA/ITMLowLevelEngine_CUDA.h
Engine/DeviceSpecific/CUDA/ITMRenTracker_CUDA.h
Engine/DeviceSpecific/CUDA/ITMSceneReconstructionEngine_CUDA.h
Engine/DeviceSpecific/CUDA/ITMSwappingEngine_CUDA.h
Engine/DeviceSpecific/CUDA/ITMViewBuilder_CUDA.h
Engine/DeviceSpecific/CUDA/ITMVisualisationEngine_CUDA.h
Engine/DeviceSpecific/CUDA/ITMMeshingEngine_CUDA.h
)

##
set(ITMLIB_OBJECTS_SOURCES
Objects/ITMPose.cpp
)

set(ITMLIB_OBJECTS_HEADERS
Objects/ITMDisparityCalib.h
Objects/ITMExtrinsics.h
Objects/ITMGlobalCache.h
Objects/ITMImageHierarchy.h
Objects/ITMIntrinsics.h
Objects/ITMLocalVBA.h
Objects/ITMPlainVoxelArray.h
Objects/ITMPointCloud.h
Objects/ITMPose.h
Objects/ITMRGBDCalib.h
Objects/ITMScene.h
Objects/ITMSceneHierarchyLevel.h
Objects/ITMSceneParams.h
Objects/ITMTemplatedHierarchyLevel.h
Objects/ITMTrackingState.h
Objects/ITMView.h
Objects/ITMViewIMU.h
Objects/ITMViewHierarchyLevel.h
Objects/ITMRenderState.h
Objects/ITMRenderState_VH.h
Objects/ITMVoxelBlockHash.h
Objects/ITMIMUMeasurement.h
Objects/ITMMesh.h
)

##
set(ITMLIB_UTILS_SOURCES
Utils/ITMCalibIO.cpp
Utils/ITMLibSettings.cpp
)

set(ITMLIB_UTILS_HEADERS
Utils/ITMCalibIO.h
Utils/ITMLibDefines.h
Utils/ITMLibSettings.h
Utils/ITMMath.h
)

#################################################################
# Collect the project files into common, CPU-only and CUDA-only #
#################################################################

set(ITMLIB_COMMON_OBJECTS
${ITMLIB_TOPLEVEL_HEADERS}
${ITMLIB_ENGINE_SOURCES}
${ITMLIB_ENGINE_HEADERS}
${ITMLIB_ENGINE_DEVICEAGNOSTIC_HEADERS}
${ITMLIB_OBJECTS_SOURCES}
${ITMLIB_OBJECTS_HEADERS}
${ITMLIB_UTILS_SOURCES}
${ITMLIB_UTILS_HEADERS}
)

set(ITMLIB_CPU_OBJECTS
${ITMLIB_ENGINE_DEVICESPECIFIC_CPU_SOURCES}
${ITMLIB_ENGINE_DEVICESPECIFIC_CPU_HEADERS}
)

set(ITMLIB_CUDA_OBJECTS
${ITMLIB_ENGINE_DEVICESPECIFIC_CUDA_SOURCES}
${ITMLIB_ENGINE_DEVICESPECIFIC_CUDA_HEADERS}
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP("" FILES ${ITMLIB_TOPLEVEL_HEADERS})
SOURCE_GROUP(Engine FILES ${ITMLIB_ENGINE_SOURCES} ${ITMLIB_ENGINE_HEADERS})
SOURCE_GROUP(Engine\\DeviceAgnostic FILES ${ITMLIB_ENGINE_DEVICEAGNOSTIC_HEADERS})
SOURCE_GROUP(Engine\\DeviceSpecific\\CPU FILES ${ITMLIB_ENGINE_DEVICESPECIFIC_CPU_SOURCES} ${ITMLIB_ENGINE_DEVICESPECIFIC_CPU_HEADERS})
SOURCE_GROUP(Engine\\DeviceSpecific\\CUDA FILES ${ITMLIB_ENGINE_DEVICESPECIFIC_CUDA_SOURCES} ${ITMLIB_ENGINE_DEVICESPECIFIC_CUDA_HEADERS})
SOURCE_GROUP(Objects FILES ${ITMLIB_OBJECTS_SOURCES} ${ITMLIB_OBJECTS_HEADERS})
SOURCE_GROUP(Utils FILES ${ITMLIB_UTILS_SOURCES} ${ITMLIB_UTILS_HEADERS})

##############################################################
# Specify the include directories, target and link libraries #
##############################################################

IF(WITH_CUDA)
  set(CUDA_ARCH "20 30" CACHE STRING "CUDA architectures to compile for")
  string(REPLACE " " ";" CUDA_ARCH_LIST ${CUDA_ARCH})
  foreach(a ${CUDA_ARCH_LIST})
    set(CUDA_ARCH_OPTIONS "${CUDA_ARCH_OPTIONS} -gencode arch=compute_${a},code=compute_${a}")
  endforeach()
  include_directories(${CUDA_INCLUDE_DIRS})
  cuda_add_library(ITMLib
	${ITMLIB_CPU_OBJECTS}
	${ITMLIB_CUDA_OBJECTS}
	${ITMLIB_COMMON_OBJECTS}
	OPTIONS "${CUDA_ARCH_OPTIONS} --use_fast_math")
  target_link_libraries(ITMLib ${CUDA_LIBRARY})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
  add_library(ITMLib ${ITMLIB_CPU_OBJECTS} ${ITMLIB_COMMON_OBJECTS})
ENDIF()

target_link_libraries(ITMLib Utils)
