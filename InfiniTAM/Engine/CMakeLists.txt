#############################
# CMakeLists.txt for Engine #
#############################

###########################
# Specify the target name #
###########################

SET(targetname Engine)

#####################################
# Offer support for input mirroring #
#####################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/OfferInputMirroring.cmake)

################################
# Specify the libraries to use #
################################

<<<<<<< HEAD
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseGLUT.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseKinect2.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenGL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenNI.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseRealSense.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseUVC.cmake)

#############################
# Specify the project files #
#############################
=======
IF(WITH_FFMPEG)
  include_directories(${FFMPEG_INCLUDE_DIR})
  add_definitions(-DCOMPILE_WITH_FFMPEG)
ENDIF()

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/OfferInputMirroring.cmake)
>>>>>>> infinitam_v3

SET(sources
ImageSourceEngine.cpp
IMUSourceEngine.cpp
Kinect2Engine.cpp
LibUVCEngine.cpp
OpenNIEngine.cpp
RealSenseEngine.cpp
)

SET(headers
ImageSourceEngine.h
IMUSourceEngine.h
Kinect2Engine.h
LibUVCEngine.h
OpenNIEngine.h
RealSenseEngine.h
FFMPEGReader.cpp
FFMPEGReader.h
FFMPEGWriter.cpp
FFMPEGWriter.h
)

<<<<<<< HEAD
#############################
# Specify the source groups #
#############################

SOURCE_GROUP("" FILES ${sources} ${headers})

##########################################
# Specify the target and where to put it #
##########################################
=======
target_link_libraries(Engine ${GLUT_LIBRARIES})
target_link_libraries(Engine ${OPENGL_LIBRARIES})
target_link_libraries(Engine ITMLib)
target_link_libraries(Engine ORUtils)
IF(WITH_CUDA)
  target_link_libraries(Engine ${CUDA_LIBRARY})
ENDIF()
IF(OPENNI_FOUND)
  target_link_libraries(Engine ${OpenNI_LIBRARY})
ENDIF(OPENNI_FOUND)
IF(DEFINED libuvc_LIBRARIES)
  target_link_libraries(Engine ${libuvc_LIBRARIES})
ENDIF()
IF(WITH_REALSENSE)
  TARGET_LINK_LIBRARIES(Engine ${RealSense_LIBRARY})
ENDIF()
IF(WITH_FFMPEG)
  target_link_libraries(Engine ${FFMPEG_LIBRARIES})
ENDIF()
>>>>>>> infinitam_v3

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)
