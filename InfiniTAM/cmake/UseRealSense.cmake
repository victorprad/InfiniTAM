######################
# UseRealSense.cmake #
######################

OPTION(WITH_REALSENSE "Build with Intel RealSense support?" OFF)

IF(MSVC_IDE AND WITH_REALSENSE)
  FIND_PATH(RealSense_ROOT attributions.rtf HINTS "C:/Program Files (x86)/Intel/RSSDK")
  FIND_PATH(RealSense_INCLUDE_DIR pxcsensemanager.h HINTS "${RealSense_ROOT}/include")
  FIND_LIBRARY(RealSense_LIBRARY libpxc HINTS "${RealSense_ROOT}/lib/x64")
ENDIF()

INCLUDE_DIRECTORIES(${RealSense_INCLUDE_DIR})
