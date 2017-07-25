###################
# UseOpenNI.cmake #
###################

OPTION(WITH_OPENNI "Build with OpenNI support?" OFF)

IF(WITH_OPENNI)
  FIND_PACKAGE(OpenNI REQUIRED)
  INCLUDE_DIRECTORIES(${OPENNI_INCLUDE_DIR})
ELSE()
  ADD_DEFINITIONS(-DCOMPILE_WITHOUT_OpenNI)
ENDIF()
