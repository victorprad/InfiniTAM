cmake_minimum_required(VERSION 2.8)
project(InfiniTAM)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

macro(CANONIFY_BOOL var)
  if (${var})
    set(${var} TRUE)
  else()
    set(${var} FALSE)
  endif()
endmacro()

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

find_package(CUDA QUIET)
CANONIFY_BOOL(CUDA_FOUND)
message(STATUS "CUDA found: " ${CUDA_FOUND})
OPTION(WITH_CUDA "Build with CUDA support?" ${CUDA_FOUND})

find_package(CSparse QUIET)
CANONIFY_BOOL(CSPARSE_FOUND)
message(STATUS "CSparse found: ${CSPARSE_FOUND}")
OPTION(WITH_CSPARSE "Build with CSparse support?" ${CSPARSE_FOUND})

find_package(OpenNI QUIET)
CANONIFY_BOOL(OPENNI_FOUND)
message(STATUS "OpenNI found: " ${OPENNI_FOUND})

find_package(libuvc QUIET)
CANONIFY_BOOL(libuvc_FOUND)
message(STATUS "libuvc found: ${libuvc_FOUND}")

find_package(RealSense QUIET)
CANONIFY_BOOL(REALSENSE_FOUND)
message(STATUS "librealsense found: ${REALSENSE_FOUND}")
OPTION(WITH_REALSENSE "Build with Intel RealSense support?" ${REALSENSE_FOUND})

find_package(PNG QUIET)
CANONIFY_BOOL(PNG_FOUND)
message(STATUS "libpng found: ${PNG_FOUND}")

find_package(FFmpeg QUIET)
CANONIFY_BOOL(FFMPEG_FOUND)
message(STATUS "FFmpeg found: ${FFMPEG_FOUND}")
OPTION(WITH_FFMPEG "Build with FFmpeg support?" ${FFMPEG_FOUND})

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)

IF(MSVC_IDE)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  add_definitions(-D_USE_MATH_DEFINES)
  add_definitions(-DUSING_CMAKE=1)
ELSE(MSVC_IDE)
  set(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
  set(CMAKE_CXX_FLAGS "-fPIC -O3 -march=native ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
  #set(CMAKE_CXX_FLAGS "-fPIC -g ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
ENDIF(MSVC_IDE)

if(APPLE)
  set(CUDA_HOST_COMPILER /usr/bin/clang)
endif(APPLE)

IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -Wno-deprecated-declarations -Wno-unused-function")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libstdc++")
ENDIF()

# If on Linux and CUDA is enabled, work around an Ubuntu 16.04 compilation error:
IF(${CMAKE_SYSTEM} MATCHES "Linux"
   AND ${WITH_CUDA}
   AND ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU"
   AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER 5.0)
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -D_FORCE_INLINES)
ENDIF()

add_subdirectory(ORUtils)
add_subdirectory(ITMLib)
add_subdirectory(RelocLib)
add_subdirectory(MiniSlamGraphLib)
add_subdirectory(InputSource)

IF(WITH_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF()

add_subdirectory(Apps)

