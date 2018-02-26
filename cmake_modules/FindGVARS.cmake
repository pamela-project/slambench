# - Try to find GVARS
#
#  GVARS_FOUND - system has GVARS
#  GVARS_INCLUDE_DIR - the GVARS include directories
#  GVARS_LIBRARY - link these to use GVARS

FIND_PATH(
  GVARS_INCLUDE_DIR
  NAMES gvars3/config.h
  PATHS
    /usr/include
    /usr/local/include
)

FIND_LIBRARY(
  GVARS_LIBRARY
  NAMES GVars3
  PATHS
    /usr/lib
    /usr/local/lib
) 

IF(GVARS_INCLUDE_DIR AND GVARS_LIBRARY)
  SET(GVARS_FOUND TRUE)
ENDIF()

IF(GVARS_FOUND)
   IF(NOT GVARS_FIND_QUIETLY)
      MESSAGE(STATUS "Found GVARS: ${GVARS_LIBRARY}")
   ENDIF()
ELSE()
   IF(GVARS_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find GVARS")
   ENDIF()
ENDIF()
