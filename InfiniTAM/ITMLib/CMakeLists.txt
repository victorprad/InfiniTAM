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
Engine/ITMDepthTracker.cpp
Engine/ITMMainEngine.cpp
Engine/ITMRenTracker.cpp
Engine/ITMTrackerFactory.cpp
Engine/ITMVisualisationEngine.cpp
)

SET(ITMLIB_ENGINE_HEADERS
Engine/ITMColorTracker.h
Engine/ITMDepthTracker.h
Engine/ITMLowLevelEngine.h
Engine/ITMMainEngine.h
Engine/ITMRenTracker.h
Engine/ITMSceneReconstructionEngine.h
Engine/ITMSwappingEngine.h
Engine/ITMTracker.h
Engine/ITMTrackerFactory.h
Engine/ITMVisualisationEngine.h
)

##
set(ITMLIB_ENGINE_DEVICEAGNOSTIC_HEADERS
Engine/DeviceAgnostic/ITMColorTracker.h
Engine/DeviceAgnostic/ITMDepthTracker.h
Engine/DeviceAgnostic/ITMLowLevelEngine.h
Engine/DeviceAgnostic/ITMRenTracker.h
Engine/DeviceAgnostic/ITMRepresentationAccess.h
Engine/DeviceAgnostic/ITMSceneReconstructionEngine.h
Engine/DeviceAgnostic/ITMSwappingEngine.h
Engine/DeviceAgnostic/ITMVisualisationEngine.h
)

##
set(ITMLIB_ENGINE_DEVICESPECIFIC_CPU_SOURCES
Engine/DeviceSpecific/CPU/ITMColorTracker_CPU.cpp
Engine/DeviceSpecific/CPU/ITMDepthTracker_CPU.cpp
Engine/DeviceSpecific/CPU/ITMLowLevelEngine_CPU.cpp
Engine/DeviceSpecific/CPU/ITMRenTracker_CPU.cpp
Engine/DeviceSpecific/CPU/ITMSceneReconstructionEngine_CPU.cpp
Engine/DeviceSpecific/CPU/ITMSwappingEngine_CPU.cpp
Engine/DeviceSpecific/CPU/ITMVisualisationEngine_CPU.cpp
)

set(ITMLIB_ENGINE_DEVICESPECIFIC_CPU_HEADERS
Engine/DeviceSpecific/CPU/ITMColorTracker_CPU.h
Engine/DeviceSpecific/CPU/ITMDepthTracker_CPU.h
Engine/DeviceSpecific/CPU/ITMLowLevelEngine_CPU.h
Engine/DeviceSpecific/CPU/ITMRenTracker_CPU.h
Engine/DeviceSpecific/CPU/ITMSceneReconstructionEngine_CPU.h
Engine/DeviceSpecific/CPU/ITMSwappingEngine_CPU.h
Engine/DeviceSpecific/CPU/ITMVisualisationEngine_CPU.h
)

##
set(ITMLIB_ENGINE_DEVICESPECIFIC_CUDA_SOURCES
Engine/DeviceSpecific/CUDA/ITMColorTracker_CUDA.cu
Engine/DeviceSpecific/CUDA/ITMDepthTracker_CUDA.cu
Engine/DeviceSpecific/CUDA/ITMLowLevelEngine_CUDA.cu
Engine/DeviceSpecific/CUDA/ITMRenTracker_CUDA.cu
Engine/DeviceSpecific/CUDA/ITMSceneReconstructionEngine_CUDA.cu
Engine/DeviceSpecific/CUDA/ITMSwappingEngine_CUDA.cu
Engine/DeviceSpecific/CUDA/ITMVisualisationEngine_CUDA.cu
)

set(ITMLIB_ENGINE_DEVICESPECIFIC_CUDA_HEADERS
Engine/DeviceSpecific/CUDA/ITMColorTracker_CUDA.h
Engine/DeviceSpecific/CUDA/ITMCUDADefines.h
Engine/DeviceSpecific/CUDA/ITMCUDAUtils.h
Engine/DeviceSpecific/CUDA/ITMDepthTracker_CUDA.h
Engine/DeviceSpecific/CUDA/ITMLowLevelEngine_CUDA.h
Engine/DeviceSpecific/CUDA/ITMRenTracker_CUDA.h
Engine/DeviceSpecific/CUDA/ITMSceneReconstructionEngine_CUDA.h
Engine/DeviceSpecific/CUDA/ITMSwappingEngine_CUDA.h
Engine/DeviceSpecific/CUDA/ITMVisualisationEngine_CUDA.h
)

##
set(ITMLIB_OBJECTS_SOURCES
Objects/ITMDisparityCalib.cpp
Objects/ITMIntrinsics.cpp
Objects/ITMPose.cpp
)

set(ITMLIB_OBJECTS_HEADERS
Objects/ITMDisparityCalib.h
Objects/ITMExtrinsics.h
Objects/ITMGlobalCache.h
Objects/ITMHashTable.h
Objects/ITMImage.h
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
Objects/ITMViewHierarchyLevel.h
Objects/ITMVisualisationState.h
Objects/ITMVoxelBlockHash.h
)

##
set(ITMLIB_UTILS_SOURCES
Utils/ITMCalibIO.cpp
Utils/ITMLibSettings.cpp
)

set(ITMLIB_UTILS_HEADERS
Utils/ITMCalibIO.h
Utils/ITMCholesky.h
Utils/ITMLibDefines.h
Utils/ITMLibSettings.h
Utils/ITMMath.h
Utils/ITMMatrix.h
Utils/ITMPixelUtils.h
Utils/ITMVector.h
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
  include_directories(${CUDA_INCLUDE_DIRS})
  cuda_add_library(ITMLib
	${ITMLIB_CPU_OBJECTS}
	${ITMLIB_CUDA_OBJECTS}
	${ITMLIB_COMMON_OBJECTS}
	OPTIONS -gencode arch=compute_11,code=compute_11 -gencode arch=compute_30,code=compute_30)
  target_link_libraries(ITMLib ${CUDA_LIBRARY})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
  add_library(ITMLib ${ITMLIB_CPU_OBJECTS} ${ITMLIB_COMMON_OBJECTS})
ENDIF()

target_link_libraries(ITMLib Utils)
