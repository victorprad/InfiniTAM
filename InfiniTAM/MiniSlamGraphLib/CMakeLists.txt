#######################################
# CMakeLists.txt for MiniSlamGraphLib #
#######################################

###########################
# Specify the target name #
###########################

SET(targetname MiniSlamGraphLib)

################################
# Specify the libraries to use #
################################

# FIXME: Remove.
IF(WITH_CSPARSE)
  include_directories(${CSPARSE_INCLUDE_DIRS})
  add_definitions(-DCOMPILE_WITH_CSPARSE)
ENDIF()

#############################
# Specify the project files #
#############################

SET(sources
GraphEdge.cpp
GraphEdgeSE3.cpp
LevenbergMarquardtMethod.cpp
MatrixWrapper.cpp
PoseGraph.cpp
SlamGraph.cpp
SlamGraphErrorFunction.cpp
QuaternionHelpers.cpp
)

SET(headers
GraphEdge.h
GraphEdgeSE3.h
GraphNode.h
GraphNodeSE3.h
LevenbergMarquardtMethod.h
Matrix_CSparse.h
MatrixWrapper.h
ParameterIndex.h
PoseGraph.h
QuaternionHelpers.h
SlamGraph.h
SlamGraphErrorFunction.h
SparseBlockMatrix.h
SparseRegularBlockMatrix.h
VariableLengthVector.h
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP("" FILES ${sources} ${headers})

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetLibTarget.cmake)

# FIXME: Remove.
target_link_libraries(${targetname} ORUtils ${CSparse_LIBRARIES})
