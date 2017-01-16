mark_as_advanced(FORCE LibRoyale_INCLUDE_DIR LibRoyale_LIBRARY LibRoyale_ROOT)

find_path(LibRoyale_INCLUDE_DIR royale.hpp 
	PATHS ${LibRoyale_INCLUDE_DIR} "C:/Program Files/royale/2.3.0.92/include" "/usr/local/include")

find_path(LibRoyale_ROOT royale_license.txt 
	PATHS ${LibRoyale_ROOT} "C:/Program Files/royale/2.3.0.92" "/usr/local")
	
find_library(LibRoyale_LIBRARY
	NAMES royale
	PATHS "C:/Program Files/royale/2.3.0.92/lib" "/usr/local/bin" ${CMAKE_LIB_PATH}
)

if (LibRoyale_LIBRARY AND LibRoyale_INCLUDE_DIR AND LibRoyale_ROOT)
	set(LibRoyale_FOUND TRUE)
else ()
	set(LibRoyale_FOUND FALSE)
endif()