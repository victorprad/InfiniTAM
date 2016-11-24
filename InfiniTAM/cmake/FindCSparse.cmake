# - Find CSparse
# This module defines
#  CSparse_INCLUDE_DIR, where to find CSparse include files
#  CSparse_LIBRARIES, the libraries needed to use CSparse
#  CSparse_FOUND, If false, do not try to use CSparse.
# also defined, but not for general use are
#  CSparse_LIBRARY, where to find the CSparse library.

set(CSparse_ROOT "/usr" CACHE FILEPATH "Root directory of CSparse")

find_library(CSparse_LIBRARY NAMES cxsparse csparse PATHS "${CSparse_ROOT}/Lib" "C:/Program Files (x86)/CSparse/Lib" "C:/Program Files/CSparse/Lib" "${CSparse_ROOT}/Bin/x64-Release/" ${CMAKE_LIB_PATH})

find_path(CSparse_INCLUDE_DIR cs.h PATH "${CSparse_ROOT}/include/suitesparse" "${CSparse_ROOT}/include")

find_package_handle_standard_args(CSparse DEFAULT_MSG CSparse_LIBRARY CSparse_INCLUDE_DIR)

if(CSPARSE_FOUND)
  set(CSparse_LIBRARIES ${CSparse_LIBRARY})
endif()

mark_as_advanced(CSparse_LIBRARY CSparse_INCLUDE_DIR)

