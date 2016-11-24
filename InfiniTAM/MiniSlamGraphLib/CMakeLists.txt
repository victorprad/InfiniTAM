IF(WITH_CSPARSE)
  include_directories(${CSPARSE_INCLUDE_DIRS})
  add_definitions(-DCOMPILE_WITH_CSPARSE)
ENDIF()

add_library(MiniSlamGraphLib
GraphEdge.h 
GraphEdgeSE3.h 
GraphNode.h GraphNodeSE3.h
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
SlamGraphErrorFunction.cpp
SlamGraph.cpp
QuaternionHelpers.cpp
PoseGraph.cpp
MatrixWrapper.cpp
LevenbergMarquardtMethod.cpp
GraphEdge.cpp
GraphEdgeSE3.cpp
)

target_link_libraries(MiniSlamGraphLib ORUtils ${CSparse_LIBRARIES})

#add_executable(SlamGraphTest progs/SlamGraphTest.cpp)
#target_link_libraries(SlamGraphTest MiniSlamGraphLib)

