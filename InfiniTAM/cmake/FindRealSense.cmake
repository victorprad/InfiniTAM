# - Find OpenNI
# This module defines
#  OpenNI_INCLUDE_DIR, where to find OpenNI include files
#  OpenNI_LIBRARIES, the libraries needed to use OpenNI
#  OpenNI_FOUND, If false, do not try to use OpenNI.
# also defined, but not for general use are
#  OpenNI_LIBRARY, where to find the OpenNI library.

SET(REALSENSE_ROOT "/usr/local" CACHE FILEPATH "Root directory of librealsense")
SET(REALSENSE_LOCALROOT "${PROJECT_SOURCE_DIR}/librealsense" CACHE FILEPATH "Root directory of local librealsense (if built)")

#  FIND_PATH(RealSense_ROOT librealsense.vc12 HINTS "D:/Develop/intel/librealsense")
FIND_PATH(RealSense_INCLUDE_DIR librealsense HINTS "${REALSENSE_ROOT}/include" "${REALSENSE_LOCALROOT}/include")
FIND_LIBRARY(RealSense_LIBRARY realsense HINTS "${REALSENSE_ROOT}/bin/x64" "${REALSENSE_ROOT}/lib" "${REALSENSE_LOCALROOT}/bin/x64" "${REALSENSE_LOCALROOT}/build" "${REALSENSE_LOCALROOT}/lib")

# handle the QUIETLY and REQUIRED arguments and set REALSENSE_FOUND to TRUE if
# all listed variables are TRUE
#include(${CMAKE_CURRENT_LIST_DIR}/FindPackageHandleStandardArgs.cmake)
#include(${CMAKE_MODULE_PATH}/FindPackageHandleStandardArgs.cmake)
find_package_handle_standard_args(RealSense DEFAULT_MSG RealSense_LIBRARY RealSense_INCLUDE_DIR)

#if(OPENNI_FOUND)
#  set(OpenNI_LIBRARIES ${OpenNI_LIBRARY})
#endif()

MARK_AS_ADVANCED(RealSense_LIBRARY RealSense_INCLUDE_DIR)
