##########################
# SetCUDALibTarget.cmake #
##########################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/Flags.cmake)

IF(WITH_CUDA)
  CUDA_ADD_LIBRARY(${targetname} STATIC ${sources} ${headers} ${templates})
ELSE()
  ADD_LIBRARY(${targetname} STATIC ${sources} ${headers} ${templates})
ENDIF()
