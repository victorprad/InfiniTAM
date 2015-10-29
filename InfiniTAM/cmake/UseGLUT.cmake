###################
# UseGLUT.cmake #
###################

FIND_PACKAGE(GLUT REQUIRED)

IF(GLUT_FOUND)
  INCLUDE_DIRECTORIES(${GLUT_INCLUDE_DIR})
ENDIF()
