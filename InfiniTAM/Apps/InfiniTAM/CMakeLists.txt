#####################################
# CMakeLists.txt for Apps/InfiniTAM #
#####################################

include_directories(${GLUT_INCLUDE_DIR})

add_executable(InfiniTAM InfiniTAM.cpp UIEngine.cpp UIEngine.h)
target_link_libraries(InfiniTAM Engine)
target_link_libraries(InfiniTAM Utils)
