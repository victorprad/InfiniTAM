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

##########################################
# Specify the target and where to put it #
##########################################

CUDA_ADD_LIBRARY(${targetname}
PoseDatabase.h PoseDatabase.cpp
LCDDatabase.h LCDDatabase.cpp
FernConservatory.h FernConservatory.cpp
LoopClosureDetector.h LoopClosureDetector.cpp
)