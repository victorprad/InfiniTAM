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
  add_definitions(-DCOMPILE_WITHOUT_Kinect2)
ENDIF(MSKINECTAPI_FOUND)

IF(DEFINED libuvc_INCLUDE_DIRS)
  include_directories(${libuvc_INCLUDE_DIRS})
  add_definitions(-DCOMPILE_WITH_LibUVC)
ENDIF()

IF(WITH_REALSENSE)
  add_definitions(-DCOMPILE_WITH_RealSenseWindows)
ENDIF()

add_library(InputSource
ImageSourceEngine.cpp
ImageSourceEngine.h
IMUSourceEngine.cpp
IMUSourceEngine.h
Kinect2Engine.cpp
Kinect2Engine.h
OpenNIEngine.cpp
OpenNIEngine.h
LibUVCEngine.cpp
LibUVCEngine.h
RealSenseEngine.cpp
RealSenseEngine.h
)

target_link_libraries(InputSource ${GLUT_LIBRARIES})
target_link_libraries(InputSource ${OPENGL_LIBRARIES})
target_link_libraries(InputSource ITMLib)
target_link_libraries(InputSource ORUtils)
IF(WITH_CUDA)
  target_link_libraries(InputSource ${CUDA_LIBRARY})
ENDIF()
IF(OPENNI_FOUND)
  target_link_libraries(InputSource ${OpenNI_LIBRARY})
ENDIF(OPENNI_FOUND)
IF(DEFINED libuvc_LIBRARIES)
  target_link_libraries(InputSource ${libuvc_LIBRARIES})
ENDIF()
IF(WITH_REALSENSE)
  TARGET_LINK_LIBRARIES(InputSource ${RealSense_LIBRARY})
ENDIF()

