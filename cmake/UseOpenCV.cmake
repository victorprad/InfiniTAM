###################
# UseOpenCV.cmake #
###################

OPTION(WITH_OPENCV "Build with OpenCV support?" OFF)

IF(WITH_OPENCV)
  FIND_PACKAGE(OpenCV 3.1.0 REQUIRED HINTS "${PROJECT_SOURCE_DIR}/libraries/opencv-3.1.0/build" "${PROJECT_SOURCE_DIR}/../spaint/libraries/opencv-3.1.0/build")
  INCLUDE_DIRECTORIES(BEFORE SYSTEM ${OpenCV_INCLUDE_DIRS})
  ADD_DEFINITIONS(-DWITH_OPENCV)
ENDIF()
