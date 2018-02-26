#######################
# UseRealSense2.cmake #
#######################

OPTION(WITH_REALSENSE2 "Build with Intel RealSense SDK 2 support?" OFF)

IF(WITH_REALSENSE2)
  FIND_PACKAGE(RealSense2 REQUIRED)
  INCLUDE_DIRECTORIES(${RealSense2_INCLUDE_DIR})
  ADD_DEFINITIONS(-DCOMPILE_WITH_RealSense2)
ENDIF()
