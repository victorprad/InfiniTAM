###########################
# OfferC++11Support.cmake #
###########################

OPTION(WITH_C++11 "Enable C++11 support?" OFF)

IF(WITH_C++11)
  # Note: C++11 is enabled by default for modern versions of VC++.
  IF(NOT MSVC_IDE)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  ENDIF()
ENDIF()
