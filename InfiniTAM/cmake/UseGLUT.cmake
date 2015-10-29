###################
# UseGLUT.cmake #
###################

IF(MSVC_IDE)
  FIND_PATH(GLUT_INCLUDE_DIR GL/glut.h HINTS "$ENV{HOMEPATH}/Downloads/freeglut/include")
  FIND_LIBRARY(GLUT_LIBRARY freeglut HINTS "$ENV{HOMEPATH}/Downloads/freeglut/lib/x64")
ENDIF()

INCLUDE_DIRECTORIES(${GLUT_INCLUDE_DIR})
