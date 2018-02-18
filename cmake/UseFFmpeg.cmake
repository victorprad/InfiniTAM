###################
# UseFFmpeg.cmake #
###################

OPTION(WITH_FFMPEG "Build with FFmpeg support?" OFF)

IF(WITH_FFMPEG)
  FIND_PACKAGE(FFmpeg QUIET)
  IF(${FFMPEG_FOUND})
    INCLUDE_DIRECTORIES(${FFMPEG_INCLUDE_DIR})
    ADD_DEFINITIONS(-DCOMPILE_WITH_FFMPEG)

    IF(MSVC_IDE)
      FIND_PATH(FFmpeg_SHARED_ROOT ff-prompt.bat HINTS $ENV{HOMEPATH}/Downloads/ffmpeg-20160310-git-689211d-win64-shared)
    ENDIF()
  ELSE()
    MESSAGE(FATAL_ERROR "FFmpeg not found!")
  ENDIF()
ENDIF()
