###################
# UseGLUT.cmake #
###################

IF(MSVC_IDE)
  FIND_PATH(GLUT_ROOT Readme.txt HINTS "${PROJECT_SOURCE_DIR}/freeglut")
  FIND_LIBRARY(GLUT_LIBRARY freeglut HINTS "${GLUT_ROOT}/lib/x64")
ELSEIF(APPLE)
  FIND_PATH(GLUT_ROOT include/GL/glut.h HINTS "~/Downloads/freeglut-2.8.1/build")
  FIND_LIBRARY(GLUT_LIBRARY libglut.dylib HINTS "${GLUT_ROOT}/lib")
ENDIF()

FIND_PATH(GLUT_INCLUDE_DIR GL/glut.h HINTS "${GLUT_ROOT}/include")

INCLUDE_DIRECTORIES(${GLUT_INCLUDE_DIR})
