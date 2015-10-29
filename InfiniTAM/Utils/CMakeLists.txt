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
  INCLUDE_DIRECTORIES(${PNG_INCLUDE_DIRS})
  ADD_DEFINITIONS(${PNG_DEFINITIONS})
  ADD_DEFINITIONS(-DUSE_LIBPNG)
ENDIF()

#############################
# Specify the project files #
#############################

SET(sources
FileUtils.cpp
)

SET(headers
FileUtils.h
NVTimer.h
)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)

#################################
# Specify the link dependencies #
#################################

IF(PNG_FOUND)
  TARGET_LINK_LIBRARIES(${targetname} ${PNG_LIBRARIES})
ENDIF()