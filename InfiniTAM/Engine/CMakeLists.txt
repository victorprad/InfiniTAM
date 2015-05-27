find_package(freenect2)

include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})

IF(WITH_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF()

IF(OPENNI_FOUND)
  include_directories(${OpenNI_INCLUDE_DIR})
ELSE(OPENNI_FOUND)
  add_definitions(-DCOMPILE_WITHOUT_OpenNI)
ENDIF(OPENNI_FOUND)

IF(MSKINECTAPI_FOUND)
  include_directories(${MSKINECTAPI_INCLUDE_DIR})
ELSE(MSKINECTAPI_FOUND)
  add_definitions(-DCOMPILE_WITHOUT_Kinect2SDK)
ENDIF(MSKINECTAPI_FOUND)

IF(freenect2_FOUND)
  include_directories(${freenect2_INCLUDE_DIRS})
  find_package(OpenCV REQUIRED core features2d)
  include_directories(${OpenCV_INCLUDE_DIRS})
ELSE(freenect2_FOUND)
  add_definitions(-DCOMPILE_WITHOUT_freenect2)
ENDIF(freenect2_FOUND)

add_library(Engine
ImageSourceEngine.cpp
ImageSourceEngine.h
Kinect2Engine.cpp
Kinect2Engine.h
OpenNIEngine.cpp
OpenNIEngine.h
UIEngine.cpp
UIEngine.h
)

target_link_libraries(Engine ${GLUT_LIBRARIES})
target_link_libraries(Engine ${OPENGL_LIBRARIES})
target_link_libraries(Engine ITMLib)
IF(WITH_CUDA)
  target_link_libraries(Engine ${CUDA_LIBRARY})
ENDIF()
IF(OPENNI_FOUND)
  target_link_libraries(Engine ${OpenNI_LIBRARY})
ENDIF(OPENNI_FOUND)
IF(freenect2_FOUND)
	target_link_libraries(Engine ${freenect2_LIBRARY})
	set(OpenCV_LIBS opencv_core opencv_features2d)
	target_link_libraries(Engine ${OpenCV_LIBS})
ENDIF()
