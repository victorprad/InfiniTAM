####################
# LinkFFmpeg.cmake #
####################

IF(WITH_FFMPEG)
  TARGET_LINK_LIBRARIES(${targetname} ${FFMPEG_LIBRARIES})

  IF(MSVC_IDE)
    ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${FFmpeg_SHARED_ROOT}/bin/avcodec-57.dll" "$<TARGET_FILE_DIR:${targetname}>")
    ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${FFmpeg_SHARED_ROOT}/bin/avdevice-57.dll" "$<TARGET_FILE_DIR:${targetname}>")
    ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${FFmpeg_SHARED_ROOT}/bin/avfilter-6.dll" "$<TARGET_FILE_DIR:${targetname}>")
    ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${FFmpeg_SHARED_ROOT}/bin/avformat-57.dll" "$<TARGET_FILE_DIR:${targetname}>")
    ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${FFmpeg_SHARED_ROOT}/bin/avutil-55.dll" "$<TARGET_FILE_DIR:${targetname}>")
  ENDIF()
ENDIF()
