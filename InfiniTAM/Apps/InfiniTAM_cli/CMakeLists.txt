#########################################
# CMakeLists.txt for Apps/InfiniTAM_cli #
#########################################

add_executable(InfiniTAM_cli CLIEngine.cpp CLIEngine.h InfiniTAM_cli.cpp)
target_link_libraries(InfiniTAM_cli InputSource)
target_link_libraries(InfiniTAM_cli ORUtils)