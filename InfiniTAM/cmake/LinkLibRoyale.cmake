#######################
# LinkLibRoyale.cmake #
#######################

IF(WITH_LIBROYALE)
  target_link_libraries(${targetname} ${LibRoyale_LIBRARY})
  
  IF(MSVC_IDE)
    ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${LibRoyale_ROOT}/bin/royale.dll" "$<TARGET_FILE_DIR:${targetname}>")
  ENDIF()
ENDIF()