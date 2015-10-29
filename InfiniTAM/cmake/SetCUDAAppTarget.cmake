##########################
# SetCUDAAppTarget.cmake #
##########################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/Flags.cmake)

IF(WITH_CUDA)
  CUDA_ADD_EXECUTABLE(${targetname} ${sources} ${headers} ${templates} OPTIONS --generate-code arch=compute_${CUDA_COMPUTE_CAPABILITY},code=sm_${CUDA_COMPUTE_CAPABILITY})
ELSE()
  ADD_EXECUTABLE(${targetname} ${sources} ${headers} ${templates})
ENDIF()

IF(MSVC_IDE)
  SET_TARGET_PROPERTIES(${targetname} PROPERTIES LINK_FLAGS_DEBUG "/DEBUG")
ENDIF()
