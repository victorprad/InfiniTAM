###################################
# CMakeLists.txt for FernRelocLib #
###################################

###########################
# Specify the target name #
###########################

SET(targetname FernRelocLib)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)

#############################
# Specify the project files #
#############################

SET(sources
FernConservatory.cpp
PoseDatabase.cpp
RelocDatabase.cpp
)

SET(headers
FernConservatory.h
PixelUtils.h
PoseDatabase.h
RelocDatabase.h
Relocaliser.h
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP("" FILES ${sources} ${headers})

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)
