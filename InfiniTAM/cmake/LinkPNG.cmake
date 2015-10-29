#################
# LinkPNG.cmake #
#################

IF(PNG_FOUND)
  TARGET_LINK_LIBRARIES(${targetname} ${PNG_LIBRARIES})
ENDIF()
