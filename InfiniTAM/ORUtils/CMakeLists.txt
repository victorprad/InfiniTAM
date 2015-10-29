#############################
# Specify the project files #
#############################

##
SET(ORUTILS_HEADERS
Vector.h
Matrix.h
Cholesky.h
MathUtils.h
Image.h
CUDADefines.h
LexicalCast.h
MemoryBlock.h
MemoryBlockPersister.h
PlatformIndependence.h
KeyValueConfig.h
)

#################################################################
# Collect the project files into common, CPU-only and CUDA-only #
#################################################################

set(ORUTILS_OBJECTS
KeyValueConfig.cpp
${ORUTILS_HEADERS}
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP("" FILES ${ORUTILS_OBJECTS} ${ORUTILS_HEADERS})

##############################################################
# Specify the include directories, target and link libraries #
##############################################################

add_library(ORUtils ${ORUTILS_OBJECTS})