######################
# UseRealSense.cmake #
######################

OPTION(WITH_REALSENSE "Build with Intel RealSense support?" OFF)

IF(MSVC_IDE AND WITH_REALSENSE)
  FIND_PATH(RealSense_ROOT librealsense.vc12 HINTS "D:/Develop/intel/librealsense")
  FIND_PATH(RealSense_INCLUDE_DIR librealsense HINTS "${RealSense_ROOT}/include")
  FIND_LIBRARY(RealSense_LIBRARY realsense HINTS "${RealSense_ROOT}/bin/x64")
ENDIF()

INCLUDE_DIRECTORIES(${RealSense_INCLUDE_DIR})
