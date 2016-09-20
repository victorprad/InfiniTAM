###########################
# OfferC++11Support.cmake #
###########################

OPTION(WITH_C++11 "Enable C++11 support?" OFF)

IF(WITH_C++11)
  # MSVC does not need C++11 flags
  IF(NOT MSVC_IDE)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  ENDIF()
ENDIF()
