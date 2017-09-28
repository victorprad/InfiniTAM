###############
# Flags.cmake #
###############

# If on Mac OS X:
IF(${CMAKE_SYSTEM} MATCHES "Darwin")
  # Make sure that C++11 warnings are disabled.
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-c++11-extensions")

  # Make sure that the template depth is sufficient.
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-depth=512")

  IF(${CMAKE_SYSTEM} MATCHES "Darwin-13.")
    # If on Mac OS X 10.9 (Mavericks), use the libstdc++ implementation of the C++ Standard Library and prevent C++11 code from being compiled.
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++")
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libstdc++")
    ADD_DEFINITIONS(-DNO_CPP11)
  ELSE()
    # Otherwise, use the libc++ implementation of the C++ Standard Library, and enable C++11 support.
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++ -std=c++11")
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libc++ -std=c++11")
  ENDIF()
ENDIF()

# If on Linux, make sure that C++11 support is enabled.
IF(${CMAKE_SYSTEM} MATCHES "Linux")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
ENDIF()

# If on Windows and using Visual Studio:
IF(MSVC_IDE)
  # Disable the annoying warnings about using secure CRT functions (they're Microsoft-specific, so we can't use them portably).
  ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS)

  # Prevent the definitions of min and max when including windows.h.
  ADD_DEFINITIONS(-DNOMINMAX)

  # Make sure that the maths constants are defined.
  ADD_DEFINITIONS(-D_USE_MATH_DEFINES)

  # Define a macro needed when using Boost.ASIO.
  ADD_DEFINITIONS(-D_WIN32_WINNT=0x0501)
ENDIF()
