include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})

IF(CUDA_FOUND)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE(CUDA_FOUND)
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF(CUDA_FOUND)

IF(OPENNI_FOUND)
  include_directories(${OpenNI_INCLUDE_DIR})
ELSE(OPENNI_FOUND)
  add_definitions(-DCOMPILE_WITHOUT_OpenNI)
ENDIF(OPENNI_FOUND)

add_library(Engine OpenNIEngine.cpp
	ImageSourceEngine.cpp
	UIEngine.cpp)

target_link_libraries(Engine ${GLUT_LIBRARIES})
target_link_libraries(Engine ${OPENGL_LIBRARIES})
target_link_libraries(Engine ITMLib)
IF(CUDA_FOUND)
  target_link_libraries(Engine ${CUDA_LIBRARY})
ENDIF(CUDA_FOUND)
IF(OPENNI_FOUND)
  target_link_libraries(Engine ${OpenNI_LIBRARY})
ENDIF(OPENNI_FOUND)

