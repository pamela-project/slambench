
find_path(BRISK_INCLUDE_PATH Brisk/Brisk.h
	~/usr/include
	~/usr/.local/include
	~/.local/include
	~/usr/local/include
	/usr/include
	/usr/local/include
	thirdparty
)

if(BRISK_INCLUDE_PATH)
	set(BRISK_FOUND TRUE)
	set(BRISK_INCLUDE_PATHS ${BRISK_INCLUDE_PATH} CACHE STRING "The include paths needed to use BRISK")
endif()

mark_as_advanced(
	BRISK_INCLUDE_PATHS
)


# Generate appropriate messages
if(BRISK_FOUND)
    if(NOT BRISK_FIND_QUIETLY)
    	   message("-- Found Brisk: ${BRISK_INCLUDE_PATH}")
    endif(NOT BRISK_FIND_QUIETLY)
else(BRISK_FOUND)
    if(BRISK_FIND_REQUIRED)
	message(FATAL_ERROR "-- Could NOT find BRISK (missing: BRISK_INCLUDE_PATH)")
    endif(BRISK_FIND_REQUIRED)
endif(BRISK_FOUND)
