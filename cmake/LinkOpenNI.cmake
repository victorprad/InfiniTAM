####################
# LinkOpenNI.cmake #
####################

IF(WITH_OPENNI)
  TARGET_LINK_LIBRARIES(${targetname} ${OPENNI_LIBRARY})

  IF(MSVC_IDE)
    ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OPENNI_ROOT}/Redist/OpenNI2.dll" "$<TARGET_FILE_DIR:${targetname}>")
  ELSE()
    ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENNI_LIBRARY} $<TARGET_FILE_DIR:${targetname}>)
  ENDIF()
  ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${OPENNI_ROOT}/Redist/OpenNI2" "$<TARGET_FILE_DIR:${targetname}>/OpenNI2")
ENDIF()
