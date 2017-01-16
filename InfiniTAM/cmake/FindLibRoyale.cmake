mark_as_advanced(FORCE LibRoyale_INCLUDE_DIR LibRoyale_LIBRARY)

find_path(LibRoyale_INCLUDE_DIR royale.hpp 
	PATHS ${LibRoyale_INCLUDE_DIR} "C:/Program Files/royale/2.3.0.92/include" "/usr/local/include")

find_library(LibRoyale_LIBRARY
	NAMES royale
	PATHS "C:/Program Files/royale/2.3.0.92/lib" "/usr/local/bin" ${CMAKE_LIB_PATH}
)

if (LibRoyale_LIBRARY AND LibRoyale_INCLUDE_DIR)
	set(LibRoyale_FOUND TRUE)
else ()
	set(LibRoyale_FOUND FALSE)
endif()