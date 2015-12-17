#############################
# Specify the project files #
#############################

##
SET(ITMLIB_TOPLEVEL_HEADERS
ITMLib.h
)

##
SET(ITMLIB_ENGINE_SOURCES
Engine/ITMDenseMapper.cpp
Engine/ITMBasicEngine.cpp
Engine/ITMMultiEngine.cpp
Engine/ITMActiveSceneManager.cpp
Engine/ITMTrackingController.cpp
Engine/ITMVisualisationEngine.cpp
)

SET(ITMLIB_ENGINE_HEADERS
Engine/ITMDenseMapper.h
Engine/ITMIMUCalibrator.h
Engine/ITMLowLevelEngine.h
Engine/ITMMainEngine.h
Engine/ITMBasicEngine.h
Engine/ITMMultiEngine.h
Engine/ITMActiveSceneManager.h
Engine/ITMSceneReconstructionEngine.h
Engine/ITMSwappingEngine.h
Engine/ITMTrackingController.h
Engine/ITMViewBuilder.h
Engine/ITMVisualisationEngine.h
)

##
set(ITMLIB_ENGINE_DEVICEAGNOSTIC_HEADERS
Engine/DeviceAgnostic/ITMLowLevelEngine.h
Engine/DeviceAgnostic/ITMRepresentationAccess.h
Engine/DeviceAgnostic/ITMSceneReconstructionEngine.h
Engine/DeviceAgnostic/ITMSwappingEngine.h
Engine/DeviceAgnostic/ITMViewBuilder.h
Engine/DeviceAgnostic/ITMVisualisationEngine.h
)

##
set(ITMLIB_ENGINE_DEVICESPECIFIC_CPU_SOURCES
Engine/DeviceSpecific/CPU/ITMLowLevelEngine_CPU.cpp
Engine/DeviceSpecific/CPU/ITMSceneReconstructionEngine_CPU.cpp
Engine/DeviceSpecific/CPU/ITMSwappingEngine_CPU.cpp
Engine/DeviceSpecific/CPU/ITMViewBuilder_CPU.cpp
Engine/DeviceSpecific/CPU/ITMVisualisationEngine_CPU.cpp
)

set(ITMLIB_ENGINE_DEVICESPECIFIC_CPU_HEADERS
Engine/DeviceSpecific/CPU/ITMLowLevelEngine_CPU.h
Engine/DeviceSpecific/CPU/ITMSceneReconstructionEngine_CPU.h
Engine/DeviceSpecific/CPU/ITMSwappingEngine_CPU.h
Engine/DeviceSpecific/CPU/ITMViewBuilder_CPU.h
Engine/DeviceSpecific/CPU/ITMVisualisationEngine_CPU.h
)

##
set(ITMLIB_ENGINE_DEVICESPECIFIC_CUDA_SOURCES
Engine/DeviceSpecific/CUDA/ITMLowLevelEngine_CUDA.cu
Engine/DeviceSpecific/CUDA/ITMSceneReconstructionEngine_CUDA.cu
Engine/DeviceSpecific/CUDA/ITMSwappingEngine_CUDA.cu
Engine/DeviceSpecific/CUDA/ITMViewBuilder_CUDA.cu
Engine/DeviceSpecific/CUDA/ITMVisualisationEngine_CUDA.cu
)

set(ITMLIB_ENGINE_DEVICESPECIFIC_CUDA_HEADERS
Engine/DeviceSpecific/CUDA/ITMLowLevelEngine_CUDA.h
Engine/DeviceSpecific/CUDA/ITMSceneReconstructionEngine_CUDA.h
Engine/DeviceSpecific/CUDA/ITMSwappingEngine_CUDA.h
Engine/DeviceSpecific/CUDA/ITMViewBuilder_CUDA.h
Engine/DeviceSpecific/CUDA/ITMVisualisationEngine_CUDA.h
)

##
SET(ITMLIB_MESHING_BASE_HEADERS
Meshing/Base/ITMMesh.h
)

##
SET(ITMLIB_MESHING_CPU_SOURCES
Meshing/CPU/ITMMeshingEngine_CPU.cpp
)

SET(ITMLIB_MESHING_CPU_HEADERS
Meshing/CPU/ITMMeshingEngine_CPU.h
)

##
SET(ITMLIB_MESHING_CUDA_SOURCES
Meshing/CUDA/ITMMeshingEngine_CUDA.cu
)

SET(ITMLIB_MESHING_CUDA_HEADERS
Meshing/CUDA/ITMMeshingEngine_CUDA.h
)

##
SET(ITMLIB_MESHING_INTERFACE_HEADERS
Meshing/Interface/ITMMeshingEngine.h
)

##
SET(ITMLIB_MESHING_SHARED_HEADERS
Meshing/Shared/ITMMeshingEngine_Shared.h
)

##
set(ITMLIB_OBJECTS_SOURCES
Objects/ITMPose.cpp
Objects/ITMLocalScene.cpp
)

set(ITMLIB_OBJECTS_HEADERS
Objects/ITMDisparityCalib.h
Objects/ITMExtrinsics.h
Objects/ITMGlobalCache.h
Objects/ITMHashEntry.h
Objects/ITMHashSwapState.h
Objects/ITMIntrinsics.h
Objects/ITMLocalScene.h
Objects/ITMLocalVBA.h
Objects/ITMPlainVoxelArray.h
Objects/ITMPointCloud.h
Objects/ITMPose.h
Objects/ITMRGBDCalib.h
Objects/ITMScene.h
Objects/ITMSceneParams.h
Objects/ITMView.h
Objects/ITMViewIMU.h
Objects/ITMRenderState.h
Objects/ITMRenderState_VH.h
Objects/ITMVoxelBlockHash.h
Objects/ITMIMUMeasurement.h
)

##
SET(ITMLIB_TRACKERS_SOURCES
Trackers/ITMTrackerFactory.cpp
)

SET(ITMLIB_TRACKERS_HEADERS
Trackers/ITMTrackerFactory.h
)

##
SET(ITMLIB_TRACKERS_BASE_HEADERS
Trackers/Base/ITMImageHierarchy.h
Trackers/Base/ITMSceneHierarchyLevel.h
Trackers/Base/ITMTemplatedHierarchyLevel.h
Trackers/Base/ITMTrackingState.h
Trackers/Base/ITMViewHierarchyLevel.h
)

##
SET(ITMLIB_TRACKERS_CPU_SOURCES
Trackers/CPU/ITMColorTracker_CPU.cpp
Trackers/CPU/ITMDepthTracker_CPU.cpp
Trackers/CPU/ITMRenTracker_CPU.cpp
Trackers/CPU/ITMWeightedICPTracker_CPU.cpp
)

SET(ITMLIB_TRACKERS_CPU_HEADERS
Trackers/CPU/ITMColorTracker_CPU.h
Trackers/CPU/ITMDepthTracker_CPU.h
Trackers/CPU/ITMRenTracker_CPU.h
Trackers/CPU/ITMWeightedICPTracker_CPU.h
)

##
SET(ITMLIB_TRACKERS_CUDA_SOURCES
Trackers/CUDA/ITMColorTracker_CUDA.cu
Trackers/CUDA/ITMDepthTracker_CUDA.cu
Trackers/CUDA/ITMRenTracker_CUDA.cu
Trackers/CUDA/ITMWeightedICPTracker_CUDA.cu
)

SET(ITMLIB_TRACKERS_CUDA_HEADERS
Trackers/CUDA/ITMColorTracker_CUDA.h
Trackers/CUDA/ITMDepthTracker_CUDA.h
Trackers/CUDA/ITMRenTracker_CUDA.h
Trackers/CUDA/ITMWeightedICPTracker_CUDA.h
)

##
SET(ITMLIB_TRACKERS_INTERFACE_SOURCES
Trackers/Interface/ITMColorTracker.cpp
Trackers/Interface/ITMDepthTracker.cpp
Trackers/Interface/ITMIMUTracker.cpp
Trackers/Interface/ITMRenTracker.cpp
Trackers/Interface/ITMWeightedICPTracker.cpp
)

SET(ITMLIB_TRACKERS_INTERFACE_HEADERS
Trackers/Interface/ITMColorTracker.h
Trackers/Interface/ITMCompositeTracker.h
Trackers/Interface/ITMDepthTracker.h
Trackers/Interface/ITMIMUTracker.h
Trackers/Interface/ITMRenTracker.h
Trackers/Interface/ITMTracker.h
Trackers/Interface/ITMWeightedICPTracker.h
)

##
SET(ITMLIB_TRACKERS_SHARED_HEADERS
Trackers/Shared/ITMColorTracker_Shared.h
Trackers/Shared/ITMDepthTracker_Shared.h
Trackers/Shared/ITMRenTracker_Shared.h
Trackers/Shared/ITMWeightedICPTracker_Shared.h
)

##
SET(ITMLIB_UTILS_SOURCES
Utils/ITMCalibIO.cpp
Utils/ITMLibSettings.cpp
)

SET(ITMLIB_UTILS_HEADERS
Utils/ITMCalibIO.h
Utils/ITMCUDAUtils.h
Utils/ITMLibDefines.h
Utils/ITMLibSettings.h
Utils/ITMMath.h
Utils/ITMPixelUtils.h
Utils/ITMVoxelTypes.h
)

#################################################################
# Collect the project files into common, CPU-only and CUDA-only #
#################################################################

set(ITMLIB_COMMON_OBJECTS
${ITMLIB_TOPLEVEL_HEADERS}
${ITMLIB_ENGINE_SOURCES}
${ITMLIB_ENGINE_HEADERS}
${ITMLIB_ENGINE_DEVICEAGNOSTIC_HEADERS}
${ITMLIB_MESHING_BASE_HEADERS}
${ITMLIB_MESHING_INTERFACE_HEADERS}
${ITMLIB_MESHING_SHARED_HEADERS}
${ITMLIB_OBJECTS_SOURCES}
${ITMLIB_OBJECTS_HEADERS}
${ITMLIB_TRACKERS_SOURCES}
${ITMLIB_TRACKERS_HEADERS}
${ITMLIB_TRACKERS_BASE_HEADERS}
${ITMLIB_TRACKERS_INTERFACE_SOURCES}
${ITMLIB_TRACKERS_INTERFACE_HEADERS}
${ITMLIB_TRACKERS_SHARED_HEADERS}
${ITMLIB_UTILS_SOURCES}
${ITMLIB_UTILS_HEADERS}
)

set(ITMLIB_CPU_OBJECTS
${ITMLIB_ENGINE_DEVICESPECIFIC_CPU_SOURCES}
${ITMLIB_ENGINE_DEVICESPECIFIC_CPU_HEADERS}
${ITMLIB_MESHING_CPU_SOURCES}
${ITMLIB_MESHING_CPU_HEADERS}
${ITMLIB_TRACKERS_CPU_SOURCES}
${ITMLIB_TRACKERS_CPU_HEADERS}
)

set(ITMLIB_CUDA_OBJECTS
${ITMLIB_ENGINE_DEVICESPECIFIC_CUDA_SOURCES}
${ITMLIB_ENGINE_DEVICESPECIFIC_CUDA_HEADERS}
${ITMLIB_MESHING_CUDA_SOURCES}
${ITMLIB_MESHING_CUDA_HEADERS}
${ITMLIB_TRACKERS_CUDA_SOURCES}
${ITMLIB_TRACKERS_CUDA_HEADERS}
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP("" FILES ${ITMLIB_TOPLEVEL_HEADERS})
SOURCE_GROUP(Engine FILES ${ITMLIB_ENGINE_SOURCES} ${ITMLIB_ENGINE_HEADERS})
SOURCE_GROUP(Engine\\DeviceAgnostic FILES ${ITMLIB_ENGINE_DEVICEAGNOSTIC_HEADERS})
SOURCE_GROUP(Engine\\DeviceSpecific\\CPU FILES ${ITMLIB_ENGINE_DEVICESPECIFIC_CPU_SOURCES} ${ITMLIB_ENGINE_DEVICESPECIFIC_CPU_HEADERS})
SOURCE_GROUP(Engine\\DeviceSpecific\\CUDA FILES ${ITMLIB_ENGINE_DEVICESPECIFIC_CUDA_SOURCES} ${ITMLIB_ENGINE_DEVICESPECIFIC_CUDA_HEADERS})
SOURCE_GROUP(Meshing\\Base FILES ${ITMLIB_MESHING_BASE_HEADERS})
SOURCE_GROUP(Meshing\\CPU FILES ${ITMLIB_MESHING_CPU_SOURCES} ${ITMLIB_MESHING_CPU_HEADERS})
SOURCE_GROUP(Meshing\\CUDA FILES ${ITMLIB_MESHING_CUDA_SOURCES} ${ITMLIB_MESHING_CUDA_HEADERS})
SOURCE_GROUP(Meshing\\Interface FILES ${ITMLIB_MESHING_INTERFACE_HEADERS})
SOURCE_GROUP(Meshing\\Shared FILES ${ITMLIB_MESHING_SHARED_HEADERS})
SOURCE_GROUP(Objects FILES ${ITMLIB_OBJECTS_SOURCES} ${ITMLIB_OBJECTS_HEADERS})
SOURCE_GROUP(Trackers FILES ${ITMLIB_TRACKERS_SOURCES} ${ITMLIB_TRACKERS_HEADERS})
SOURCE_GROUP(Trackers\\Base FILES ${ITMLIB_TRACKERS_BASE_HEADERS})
SOURCE_GROUP(Trackers\\CPU FILES ${ITMLIB_TRACKERS_CPU_SOURCES} ${ITMLIB_TRACKERS_CPU_HEADERS})
SOURCE_GROUP(Trackers\\CUDA FILES ${ITMLIB_TRACKERS_CUDA_SOURCES} ${ITMLIB_TRACKERS_CUDA_HEADERS})
SOURCE_GROUP(Trackers\\Interface FILES ${ITMLIB_TRACKERS_INTERFACE_SOURCES} ${ITMLIB_TRACKERS_INTERFACE_HEADERS})
SOURCE_GROUP(Trackers\\Shared FILES ${ITMLIB_TRACKERS_SHARED_HEADERS})
SOURCE_GROUP(Utils FILES ${ITMLIB_UTILS_SOURCES} ${ITMLIB_UTILS_HEADERS})

##############################################################
# Specify the include directories, target and link libraries #
##############################################################

if(WITH_CUDA)
  if(NOT DEFINED CUDA_COMPUTE_CAPABILITY)
    include(${PROJECT_SOURCE_DIR}/cmake/CUDACheckCompute.cmake)
  endif()
  set(CUDA_ARCH ${CUDA_COMPUTE_CAPABILITY} CACHE STRING "CUDA architectures to compile for")
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
else()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
  add_library(ITMLib ${ITMLIB_CPU_OBJECTS} ${ITMLIB_COMMON_OBJECTS})
endif()

target_link_libraries(ITMLib Utils)
target_link_libraries(ITMLib LCDLib)
