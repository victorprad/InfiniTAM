##################
# LinkGLUT.cmake #
##################

IF(GLUT_FOUND)
  TARGET_LINK_LIBRARIES(${targetname} ${GLUT_LIBRARIES})
ENDIF()
