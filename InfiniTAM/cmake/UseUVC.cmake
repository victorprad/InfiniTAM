################
# UseUVC.cmake #
################

FIND_PACKAGE(libuvc QUIET)
CANONIFY_BOOL(libuvc_FOUND)
MESSAGE(STATUS "libuvc found: ${libuvc_FOUND}")

IF(libuvc_FOUND)
  INCLUDE_DIRECTORIES(${libuvc_INCLUDE_DIRS})
  ADD_DEFINITIONS(-DCOMPILE_WITH_LibUVC)
ENDIF()
