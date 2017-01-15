# - Find libroyale
# This module defines
#  LibRoyale_INCLUDE_DIR, where to find lib royale include files
#  LibRoyale_LIBRARIES, the libraries needed to use lib royale
#  LibRoyale_FOUND, If false, do not try to use lib royale.
# also defined, but not for general use are
#  LIB_ROYALE_ROOT, where to find the lib royale library.

set(LIB_ROYALE_ROOT "/usr/local" CACHE FILEPATH "Root directory of LibRoyale")

# Finally the library itself
find_library(LibRoyale_LIBRARY
    NAMES LibRoyale
    PATHS "${LIB_ROYALE_ROOT}/bin" ${CMAKE_LIB_PATH}
)

find_path(LibRoyale_INCLUDE_DIR royale.hpp PATH "${LIB_ROYALE_ROOT}/include")

find_library(LibRoyale_LIBRARY royale PATH "${LIB_ROYALE_ROOT}/bin")

#include(${CMAKE_CURRENT_LIST_DIR}/FindPackageHandleStandardArgs.cmake)
#include(${CMAKE_MODULE_PATH}/FindPackageHandleStandardArgs.cmake)
find_package_handle_standard_args(LibRoyale DEFAULT_MSG LibRoyale_LIBRARY LibRoyale_INCLUDE_DIR)

if(LibRoyale_FOUND)
    set(LibRoyale_LIBRARIES ${LibRoyale_LIBRARY})
endif()

mark_as_advanced(LibRoyale_LIBRARY LibRoyale_INCLUDE_DIR)

