##########################
# SetInstallTarget.cmake #
##########################

#install target
INSTALL(TARGETS ${targetname}
        DESTINATION lib/InfiniTAM/${targetname})
#install headers of target
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/. DESTINATION include/InfiniTAM/${targetname}
        FILES_MATCHING REGEX "^.*\\.(h|hpp)$"
)
