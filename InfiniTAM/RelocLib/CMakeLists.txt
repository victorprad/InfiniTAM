IF(WITH_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF()

add_library(RelocLib
PoseDatabase.h PoseDatabase.cpp
RelocDatabase.h RelocDatabase.cpp
FernConservatory.h FernConservatory.cpp
Relocaliser.h Relocaliser.cpp
)
