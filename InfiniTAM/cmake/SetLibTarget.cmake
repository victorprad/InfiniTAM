######################
# SetLibTarget.cmake #
######################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/Flags.cmake)

ADD_LIBRARY(${targetname} STATIC ${sources} ${headers} ${templates})
