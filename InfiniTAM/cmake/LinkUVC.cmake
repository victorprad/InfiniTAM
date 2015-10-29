#################
# LinkUVC.cmake #
#################

IF(libuvc_FOUND)
  TARGET_LINK_LIBRARIES(${targetname} ${libuvc_LIBRARIES})
ENDIF()
