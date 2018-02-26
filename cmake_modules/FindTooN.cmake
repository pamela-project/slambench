
find_path(TOON_INCLUDE_PATH TooN/TooN.h
	~/usr/include
	~/usr/.local/include
	~/.local/include
	~/usr/local/include
	/usr/include
	/usr/local/include
	thirdparty
)

if(TOON_INCLUDE_PATH)
	set(TooN_FOUND TRUE)
	set(TOON_INCLUDE_PATHS ${TOON_INCLUDE_PATH} CACHE STRING "The include paths needed to use TooN")
endif()

mark_as_advanced(
	TOON_INCLUDE_PATHS
)


# Generate appropriate messages
if(TooN_FOUND)
    if(NOT TooN_FIND_QUIETLY)
    	   message(STATUS "Found Toon: ${TOON_INCLUDE_PATH}")
    endif(NOT TooN_FIND_QUIETLY)
else(TooN_FOUND)
    if(TooN_FIND_REQUIRED)
	message(FATAL_ERROR "Could NOT find TooN (missing: TOON_INCLUDE_PATH)")
    endif(TooN_FIND_REQUIRED)
endif(TooN_FOUND)
