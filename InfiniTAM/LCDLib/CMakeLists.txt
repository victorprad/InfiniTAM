#############################
# CMakeLists.txt for LCDLib #
#############################

###########################
# Specify the target name #
###########################

SET(targetname LCDLib)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)

#############################
# Specify the project files #
#############################

SET(sources
FernConservatory.cpp
LCDDatabase.cpp
LoopClosureDetector.h
PoseDatabase.cpp
)

SET(headers
FernConservatory.h
LCDDatabase.h
LoopClosureDetector.cpp
PoseDatabase.h
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP("" FILES ${sources} ${headers})

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)