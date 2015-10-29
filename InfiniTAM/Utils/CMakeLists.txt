############################
# CMakeLists.txt for Utils #
############################

###########################
# Specify the target name #
###########################

SET(targetname Utils)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)

IF(PNG_FOUND)
  include_directories(${PNG_INCLUDE_DIRS})
  add_definitions(${PNG_DEFINITIONS})
  add_definitions(-DUSE_LIBPNG)
ENDIF()

CUDA_ADD_LIBRARY(Utils
FileUtils.cpp
FileUtils.h
NVTimer.h
)

IF(PNG_FOUND)
  target_link_libraries(Utils ${PNG_LIBRARIES})
ENDIF()