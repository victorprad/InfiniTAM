##############################
# CMakeLists.txt for ORUtils #
##############################

###########################
# Specify the target name #
###########################

SET(targetname ORUtils)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)

#############################
# Specify the project files #
#############################

SET(sources
KeyValueConfig.cpp
)

SET(headers
Cholesky.h
CUDADefines.h
Image.h
KeyValueConfig.h
LexicalCast.h
MathUtils.h
Matrix.h
MemoryBlock.h
MemoryBlockPersister.h
PlatformIndependence.h
Vector.h
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP("" FILES ${sources} ${headers})

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)