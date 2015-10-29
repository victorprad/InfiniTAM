###################
# UseGLUT.cmake #
###################

IF(MSVC_IDE)
  FIND_PATH(GLUT_ROOT Readme.txt HINTS "$ENV{HOMEPATH}/Downloads/freeglut")
  FIND_PATH(GLUT_INCLUDE_DIR GL/glut.h HINTS "${GLUT_ROOT}/include")
  FIND_LIBRARY(GLUT_LIBRARY freeglut HINTS "${GLUT_ROOT}/lib/x64")
ENDIF()

INCLUDE_DIRECTORIES(${GLUT_INCLUDE_DIR})
