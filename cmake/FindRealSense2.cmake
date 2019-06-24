

SET(REALSENSE2_ROOT "/usr/local" CACHE FILEPATH "Root directory of librealsense")

FIND_PATH(RealSense2_INCLUDE_DIR librealsense2/rs.hpp HINTS "${REALSENSE2_ROOT}/include")
FIND_LIBRARY(RealSense2_LIBRARY librealsense2 HINTS "${REALSENSE2_ROOT}/lib" "${REALSENSE2_ROOT}/lib/librealsense2.dylib")

# handle the QUIETLY and REQUIRED arguments and set REALSENSE2_FOUND to TRUE if
# all listed variables are TRUE
#include(${CMAKE_CURRENT_LIST_DIR}/FindPackageHandleStandardArgs.cmake)
#include(${CMAKE_MODULE_PATH}/FindPackageHandleStandardArgs.cmake)
find_package_handle_standard_args(RealSense2 DEFAULT_MSG RealSense2_LIBRARY RealSense2_INCLUDE_DIR)

#if(OPENNI_FOUND)
#  set(OpenNI_LIBRARIES ${OpenNI_LIBRARY})
#endif()

MARK_AS_ADVANCED(RealSense2_LIBRARY RealSense2_INCLUDE_DIR)
