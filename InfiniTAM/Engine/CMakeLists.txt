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

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseGLUT.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenGL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenNI.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseRealSense.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseUVC.cmake)

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

IF(MSKINECTAPI_FOUND)
  include_directories(${MSKINECTAPI_INCLUDE_DIR})
ELSE(MSKINECTAPI_FOUND)
  add_definitions(-DCOMPILE_WITHOUT_Kinect2)
ENDIF(MSKINECTAPI_FOUND)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)

#################################
# Specify the link dependencies #
#################################

TARGET_LINK_LIBRARIES(${targetname} ITMLib ORUtils)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkGLUT.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkOpenGL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkOpenNI.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkRealSense.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkUVC.cmake)