################
# UseUVC.cmake #
################

OPTION(WITH_UVC "Build with libuvc support?" OFF)

IF(WITH_UVC)
  FIND_PACKAGE(libuvc REQUIRED)

  INCLUDE_DIRECTORIES(${libuvc_INCLUDE_DIRS})
  ADD_DEFINITIONS(-DCOMPILE_WITH_LibUVC)
ENDIF()
