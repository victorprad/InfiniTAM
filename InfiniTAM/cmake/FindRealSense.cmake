# - Find OpenNI
# This module defines
#  OpenNI_INCLUDE_DIR, where to find OpenNI include files
#  OpenNI_LIBRARIES, the libraries needed to use OpenNI
#  OpenNI_FOUND, If false, do not try to use OpenNI.
# also defined, but not for general use are
#  OpenNI_LIBRARY, where to find the OpenNI library.

set(REALSENSE_ROOT "/usr/local" CACHE FILEPATH "Root directory of librealsense")

#  FIND_PATH(RealSense_ROOT librealsense.vc12 HINTS "D:/Develop/intel/librealsense")
FIND_PATH(RealSense_INCLUDE_DIR librealsense HINTS "${REALSENSE_ROOT}/include")
FIND_LIBRARY(RealSense_LIBRARY realsense HINTS "${REALSEMSE_ROOT}/bin/x64" "${REALSENSE_ROOT}/lib")

# handle the QUIETLY and REQUIRED arguments and set REALSENSE_FOUND to TRUE if
# all listed variables are TRUE
#include(${CMAKE_CURRENT_LIST_DIR}/FindPackageHandleStandardArgs.cmake)
#include(${CMAKE_MODULE_PATH}/FindPackageHandleStandardArgs.cmake)
find_package_handle_standard_args(RealSense DEFAULT_MSG RealSense_LIBRARY RealSense_INCLUDE_DIR)

#if(OPENNI_FOUND)
#  set(OpenNI_LIBRARIES ${OpenNI_LIBRARY})
#endif()

mark_as_advanced(RealSense_LIBRARY RealSense_INCLUDE_DIR)

