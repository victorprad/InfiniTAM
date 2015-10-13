#######################
# LinkRealSense.cmake #
#######################

IF(MSVC_IDE AND WITH_REALSENSE)
  TARGET_LINK_LIBRARIES(${targetname} ${RealSense_LIBRARY})
ENDIF()
