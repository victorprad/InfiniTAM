######################
# UseLibRoyale.cmake #
######################

OPTION(WITH_LIBROYALE "Build with LibRoyale support?" OFF)

IF(WITH_LIBROYALE)
  FIND_PACKAGE(LibRoyale REQUIRED)
  INCLUDE_DIRECTORIES(${LibRoyale_INCLUDE_DIR})
  ADD_DEFINITIONS(-DCOMPILE_WITH_LibRoyale)
ENDIF()
