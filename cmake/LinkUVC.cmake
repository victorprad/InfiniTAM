#################
# LinkUVC.cmake #
#################

IF(WITH_UVC)
  TARGET_LINK_LIBRARIES(${targetname} ${libuvc_LIBRARIES})
ENDIF()
