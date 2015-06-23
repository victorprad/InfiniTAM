IF(WITH_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF()

IF(PNG_FOUND)
  include_directories(${PNG_INCLUDE_DIRS})
  add_definitions(${PNG_DEFINITIONS})
  add_definitions(-DUSE_LIBPNG)
ENDIF()

add_library(Utils
FileUtils.cpp
FileUtils.h
NVTimer.h
)

IF(PNG_FOUND)
  target_link_libraries(Utils ${PNG_LIBRARIES})
ENDIF()

