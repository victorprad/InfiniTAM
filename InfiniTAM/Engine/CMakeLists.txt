#############################
# CMakeLists.txt for Engine #
#############################

###########################
# Specify the target name #
###########################

SET(targetname Engine)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenGL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenNI.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseRealSense.cmake)

#############################
# Specify the project files #
#############################

SET(sources
CLIEngine.cpp
ImageSourceEngine.cpp
IMUSourceEngine.cpp
Kinect2Engine.cpp
LibUVCEngine.cpp
OpenNIEngine.cpp
RealSenseEngine.cpp
UIEngine.cpp
)

SET(headers
CLIEngine.h
ImageSourceEngine.h
IMUSourceEngine.h
Kinect2Engine.h
LibUVCEngine.h
OpenNIEngine.h
RealSenseEngine.h
UIEngine.h
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP("" FILES ${sources} ${headers})

##########################################
# Specify additional include directories #
##########################################

include_directories(${GLUT_INCLUDE_DIR})

IF(WITH_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF()

IF(MSKINECTAPI_FOUND)
  include_directories(${MSKINECTAPI_INCLUDE_DIR})
ELSE(MSKINECTAPI_FOUND)
  add_definitions(-DCOMPILE_WITHOUT_Kinect2)
ENDIF(MSKINECTAPI_FOUND)

IF(DEFINED libuvc_INCLUDE_DIRS)
  include_directories(${libuvc_INCLUDE_DIRS})
  add_definitions(-DCOMPILE_WITH_LibUVC)
ENDIF()

##########################################
# Specify the target and where to put it #
##########################################

ADD_LIBRARY(${targetname} ${sources} ${headers})

#################################
# Specify the link dependencies #
#################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkOpenGL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkOpenNI.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkRealSense.cmake)

TARGET_LINK_LIBRARIES(${targetname} ${GLUT_LIBRARIES})
IF(WITH_CUDA)
  TARGET_LINK_LIBRARIES(${targetname} ${CUDA_LIBRARY})
ENDIF()

IF(DEFINED libuvc_LIBRARIES)
  TARGET_LINK_LIBRARIES(${targetname} ${libuvc_LIBRARIES})
ENDIF()

TARGET_LINK_LIBRARIES(${targetname} ITMLib ORUtils)