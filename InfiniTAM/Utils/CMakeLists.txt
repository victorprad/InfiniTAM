############################
# CMakeLists.txt for Utils #
############################

###########################
# Specify the target name #
###########################

SET(targetname Utils)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UsePNG.cmake)

#############################
# Specify the project files #
#############################

SET(sources
FileUtils.cpp
)

SET(headers
FileUtils.h
NVTimer.h
)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)