# -*- mode: cmake; -*-
###############################################################################
# Find Ceres
#
# This sets the following variables:
# CERES_FOUND - True if CERES was found.
# CERES_INCLUDE_DIRS - Directories containing the CERES include files.
# CERES_LIBRARIES - Libraries needed to use CERES.
#

find_path(CERES_INCLUDE_PATH ceres/ceres.h
  ${CERES_DIR}/include	
  NO_DEFAULT_PATH
)

find_library(CERES_DYNAMIC_LIBRARY NAMES libceres.so PATHS
  ${CERES_DIR}/lib64
  ${CERES_DIR}/lib
  NO_DEFAULT_PATH
)
find_library(CERES_STATIC_LIBRARY NAMES libceres.a PATHS
  ${CERES_DIR}/lib64
  ${CERES_DIR}/lib
    NO_DEFAULT_PATH
  )

message(STATUS "CERES_DIR=${CERES_DIR}")
message(STATUS "CERES_INCLUDE_PATH=${CERES_INCLUDE_PATH}")
message(STATUS "CERES_DYNAMIC_LIBRARY=${CERES_DYNAMIC_LIBRARY}")
message(STATUS "CERES_STATIC_LIBRARY=${CERES_STATIC_LIBRARY}")


if(CERES_STATIC_LIBRARY OR CERES_DYNAMIC_LIBRARY)
if(CERES_INCLUDE_PATH)
	set(CERES_FOUND TRUE)
	set(CERES_INCLUDE_PATHS ${CERES_INCLUDE_PATH} CACHE STRING "The include paths needed to use CERES")
	if(CERES_STATIC_LIBRARY)
	  set(CERES_LIBRARIES "${CERES_STATIC_LIBRARY}" CACHE STRING "The libraries needed to use CERES")
	ELSE()
	  set(CERES_LIBRARIES "${CERES_DYNAMIC_LIBRARY}" CACHE STRING "The libraries needed to use CERES")
	ENDIF()
else()
    if(Ceres_FIND_REQUIRED)
	message(FATAL_ERROR "-- Could NOT find CERES (missing: CERES_INCLUDE_PATH)")
    endif(Ceres_FIND_REQUIRED)
endif()
else()
    if(Ceres_FIND_REQUIRED)
	message(FATAL_ERROR "-- Could NOT find CERES (missing: CERES_STATIC_LIBRARY or CERES_DYNAMIC_LIBRARY)")
    endif(Ceres_FIND_REQUIRED)
endif()

mark_as_advanced(
  CERES_INCLUDE_PATHS
  CERES_LIBRARIES
)



# Generate appropriate messages
if(Ceres_FOUND)
    if(NOT Ceres_FIND_QUIETLY)
    	   message("-- Found CERES: ${CERES_INCLUDE_PATH}")
    endif(NOT Ceres_FIND_QUIETLY)
endif(Ceres_FOUND)
