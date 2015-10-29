#################
# LinkPNG.cmake #
#################

IF(WITH_PNG)
  TARGET_LINK_LIBRARIES(${targetname} ${PNG_LIBRARIES})
ENDIF()
