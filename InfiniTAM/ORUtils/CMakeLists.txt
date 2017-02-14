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
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UsePNG.cmake)

#############################
# Specify the project files #
#############################

SET(sources
FileUtils.cpp
KeyValueConfig.cpp
SE3Pose.cpp
)

SET(headers
Cholesky.h
CUDADefines.h
FileUtils.h
HomkerMap.h
Image.h
KeyValueConfig.h
LexicalCast.h
MathUtils.h
Matrix.h
MemoryBlock.h
MemoryBlockPersister.h
MemoryDeviceType.h
NVTimer.h
PlatformIndependence.h
SE3Pose.h
SVMClassifier.h
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
