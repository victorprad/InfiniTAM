###################
# UseOpenNI.cmake #
###################

OPTION(WITH_OPENNI "Build with OpenNI support?" OFF)

IF(WITH_OPENNI)
  IF(MSVC_IDE)
    FIND_PATH(OPENNI_ROOT LICENSE HINTS "C:/Program Files/OpenNI2")
  ELSEIF(APPLE)
    FIND_PATH(OPENNI_ROOT primesense-usb.rules HINTS ~/Downloads/OpenNI-MacOSX-x64-2.2)
  ELSEIF("${CMAKE_SYSTEM}" MATCHES "Linux")
    FIND_PATH(OPENNI_ROOT LICENSE HINTS ~/Software/OpenNI2)
  ELSE()
    MESSAGE(FATAL_ERROR "OpenNI not currently set up to work on this platform.")
  ENDIF()

  FIND_PATH(OPENNI_INCLUDE_DIR OpenNI.h HINTS "${OPENNI_ROOT}/Include")
  FIND_LIBRARY(OPENNI_LIBRARY OpenNI2 HINTS "${OPENNI_ROOT}/Bin/x64-Release" "${OPENNI_ROOT}/Lib" "${OPENNI_ROOT}/Redist")

  INCLUDE_DIRECTORIES(${OPENNI_INCLUDE_DIR})
ELSE()
  ADD_DEFINITIONS(-DCOMPILE_WITHOUT_OpenNI)
ENDIF()
