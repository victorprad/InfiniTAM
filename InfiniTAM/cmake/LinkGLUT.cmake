##################
# LinkGLUT.cmake #
##################

TARGET_LINK_LIBRARIES(${targetname} ${GLUT_LIBRARY})

IF(MSVC_IDE)
  ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${GLUT_ROOT}/bin/x64/freeglut.dll" "$<TARGET_FILE_DIR:${targetname}>")
ENDIF()
