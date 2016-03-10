####################
# LinkFFmpeg.cmake #
####################

IF(WITH_FFMPEG)
  TARGET_LINK_LIBRARIES(${targetname} ${FFMPEG_LIBRARIES})
ENDIF()
