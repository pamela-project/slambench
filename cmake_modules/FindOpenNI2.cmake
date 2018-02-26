find_library(OPENNI2_LIBRARY
        NAMES OpenNI2
    	PATHS ~/usr/lib ~/usr/local/lib ~/.local/lib64  ~/.local/lib  /usr/lib /usr/local/lib /data/Repositories/OpenNI2-2.2-beta2/Bin/x64-Release /scratch/cad/OpenNI/Redist /data/sw/OpenNI/OpenNI2-2.2-beta2/Bin/x64-Release 
        PATH_SUFFIXES openni2 ni2
	)
if(OPENNI2_LIBRARY)
else()
find_library(OPENNI2_LIBRARY
        NAMES OpenNI2
	PATHS  /usr/lib /usr/local/lib
)
endif()

find_path(OPENNI2_INCLUDE_PATH
        NAMES OpenNI.h
        PATHS ~/usr/include ~/.local/include ~/usr/local/include /usr/include /usr/local/include /scratch/cad/OpenNI/Include /data/sw/OpenNI/OpenNI2-2.2-beta2/Include 
        PATH_SUFFIXES openni2 ni2
	)


if(OPENNI2_LIBRARY)
else()
        message(STATUS "NOT Found OpenNI2 Library: ${OPENNI2_LIBRARY}")
endif()
if(OPENNI2_INCLUDE_PATH)
  set(OPENNI2_INCLUDE_DIR ${OPENNI2_INCLUDE_PATH})
else()
        message(STATUS "NOT Found OpenNI2 includes: ${OPENNI2_INCLUDE_PATH}")
endif()
if(OPENNI2_LIBRARY AND OPENNI2_INCLUDE_PATH)
        message(STATUS "Found OpenNI2: ${OPENNI2_LIBRARY}")
	set(OpenNI2_FOUND TRUE)
        set(OPENNI2_INCLUDE_PATHS ${OPENNI2_INCLUDE_PATH} CACHE STRING "The include paths needed to use OpenNI2")
        set(OPENNI2_LIBRARIES ${OPENNI2_LIBRARY} CACHE STRING "The libraries needed to use OpenNI2")
else()
    message(STATUS "NOT FOUND: OpenNI2")
endif()

mark_as_advanced(
        OPENNI2_INCLUDE_PATHS
        OPENNI2_LIBRARIES
	)
