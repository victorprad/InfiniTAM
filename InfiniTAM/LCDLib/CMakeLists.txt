IF(WITH_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF()

add_library(LCDLib
PoseDatabase.h PoseDatabase.cpp
LCDDatabase.h LCDDatabase.cpp
FernConservatory.h FernConservatory.cpp
LoopClosureDetector.h LoopClosureDetector.cpp
)
