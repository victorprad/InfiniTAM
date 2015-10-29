##########################################
# Top-level CMakeLists.txt for InfiniTAM #
##########################################

cmake_minimum_required(VERSION 2.8)

PROJECT(InfiniTAM)

#################################
# Specify the CMake module path #
#################################

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

########################
# Define useful macros #
########################

MACRO(CANONIFY_BOOL var)
  IF(${var})
    SET(${var} TRUE)
  ELSE()
    SET(${var} FALSE)
  ENDIF()
ENDMACRO()

#################################
# Add additional compiler flags #
#################################

IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -Wno-deprecated-declarations -Wno-unused-function")
ENDIF()

IF(MSVC_IDE)
  # Suppress auto-linking of libraries in InfiniTAM.
  ADD_DEFINITIONS(-DUSING_CMAKE=1)
ENDIF()

IF(NOT MSVC_IDE)
  SET(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
  SET(CMAKE_CXX_FLAGS "-fPIC -O3 -march=native ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
  #SET(CMAKE_CXX_FLAGS "-fPIC -g ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
ENDIF()

######################
# Add subdirectories #
######################

ADD_SUBDIRECTORY(ORUtils)
ADD_SUBDIRECTORY(ITMLib)
ADD_SUBDIRECTORY(LCDLib)
ADD_SUBDIRECTORY(Utils)
ADD_SUBDIRECTORY(Engine)

###############################
# Specify application targets #
###############################

SET(targetname InfiniTAM_cli)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
SET(sources InfiniTAM_cli.cpp)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDAAppTarget.cmake)
TARGET_LINK_LIBRARIES(${targetname} Engine ITMLib LCDLib ORUtils Utils)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkOpenNI.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkPNG.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkRealSense.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkUVC.cmake)

SET(targetname InfiniTAM)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
SET(sources InfiniTAM.cpp)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDAAppTarget.cmake)
TARGET_LINK_LIBRARIES(${targetname} Engine ITMLib LCDLib ORUtils Utils)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkGLUT.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkOpenGL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkOpenNI.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkPNG.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkRealSense.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkUVC.cmake)