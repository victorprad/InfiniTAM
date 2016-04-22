######################
# UseRealSense.cmake #
######################

OPTION(WITH_REALSENSE "Build with Intel RealSense support?" OFF)

IF(WITH_REALSENSE)
  FIND_PACKAGE(RealSense REQUIRED)
  INCLUDE_DIRECTORIES(${RealSense_INCLUDE_DIR})
  ADD_DEFINITIONS(-DCOMPILE_WITH_RealSense)
ENDIF()
