#######################
# LinkRealSense.cmake #
#######################

IF(WITH_REALSENSE)
  TARGET_LINK_LIBRARIES(${targetname} ${RealSense_LIBRARY})
ENDIF()
