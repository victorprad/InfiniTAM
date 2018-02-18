####################
# UseKinect2.cmake #
####################

# Note: This doesn't work yet - the code here is a starting point for future reference.

IF(MSKINECTAPI_FOUND)
  INCLUDE_DIRECTORIES(${MSKINECTAPI_INCLUDE_DIR})
ELSE(MSKINECTAPI_FOUND)
  ADD_DEFINITIONS(-DCOMPILE_WITHOUT_Kinect2)
ENDIF(MSKINECTAPI_FOUND)