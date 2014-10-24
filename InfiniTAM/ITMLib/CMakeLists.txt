set(ITMLIB_CPU_OBJECTS
	Engine/DeviceSpecific/CPU/ITMLowLevelEngine_CPU.cpp
	Engine/DeviceSpecific/CPU/ITMColorTracker_CPU.cpp
	Engine/DeviceSpecific/CPU/ITMDepthTracker_CPU.cpp
	Engine/DeviceSpecific/CPU/ITMRenTracker_CPU.cpp
	Engine/DeviceSpecific/CPU/ITMSceneReconstructionEngine_CPU.cpp
	Engine/DeviceSpecific/CPU/ITMSwappingEngine_CPU.cpp
	Engine/DeviceSpecific/CPU/ITMVisualisationEngine_CPU.cpp)
set(ITMLIB_CUDA_OBJECTS
	Engine/DeviceSpecific/CUDA/ITMLowLevelEngine_CUDA.cu
	Engine/DeviceSpecific/CUDA/ITMColorTracker_CUDA.cu
	Engine/DeviceSpecific/CUDA/ITMDepthTracker_CUDA.cu
	Engine/DeviceSpecific/CUDA/ITMRenTracker_CUDA.cu
	Engine/DeviceSpecific/CUDA/ITMSceneReconstructionEngine_CUDA.cu
	Engine/DeviceSpecific/CUDA/ITMSwappingEngine_CUDA.cu
	Engine/DeviceSpecific/CUDA/ITMVisualisationEngine_CUDA.cu)
set(ITMLIB_COMMON_OBJECTS
	Engine/ITMColorTracker.cpp
	Engine/ITMDepthTracker.cpp
	Engine/ITMRenTracker.cpp
	Engine/ITMTrackerFactory.cpp
	Engine/ITMVisualisationEngine.cpp
	Engine/ITMMainEngine.cpp
	Objects/ITMDisparityCalib.cpp
	Objects/ITMIntrinsics.cpp
	Objects/ITMPose.cpp
	Utils/ITMLibSettings.cpp
	Utils/ITMCalibIO.cpp)

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
