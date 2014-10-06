IF(CUDA_FOUND)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE(CUDA_FOUND)
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF(CUDA_FOUND)

add_library(Utils FileUtils)

