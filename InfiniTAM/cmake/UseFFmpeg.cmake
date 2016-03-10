###################
# UseFFmpeg.cmake #
###################

OPTION(WITH_FFMPEG "Build with FFmpeg support?" OFF)

IF(WITH_FFMPEG)
  FIND_PACKAGE(FFmpeg QUIET)
  IF(${FFMPEG_FOUND})
    INCLUDE_DIRECTORIES(${FFMPEG_INCLUDE_DIR})
    ADD_DEFINITIONS(-DCOMPILE_WITH_FFMPEG)
  ELSE()
    MESSAGE(FATAL_ERROR "FFmpeg not found!")
  ENDIF()
ENDIF()
