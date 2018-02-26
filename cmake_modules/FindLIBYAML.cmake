# CMake module to search for the libyaml library
# (library for parsing YAML files)
#
# If it's found it sets LIBYAML_FOUND to TRUE
# and following variables are set:
#    LIBYAML_INCLUDE_DIR
#    LIBYAML_LIBRARY


FIND_PATH(LIBYAML_INCLUDE_DIR NAMES yaml.h 
	HINTS 
	/usr/include/yaml-cpp/)
FIND_LIBRARY(LIBYAML_LIBRARY NAMES yaml-cpp
	HINTS
	/usr/lib64)

if(LIBYAML_LIBRARY)
  message(STATUS "Found LIBYAML Library: ${LIBYAML_LIBRARY}")
else()
  message(STATUS "NOT Found LIBYAML Library: ${LIBYAML_LIBRARY}")
endif()

if(LIBYAML_INCLUDE_DIR)
  message(STATUS "Found LIBYAML includes: ${LIBYAML_INCLUDE_DIR}")
else()
  message(STATUS "NOT Found LIBYAML includes: ${LIBYAML_INCLUDE_DIR}")
endif()

if(LIBYAML_LIBRARY AND LIBYAML_INCLUDE_DIR)
  message(STATUS "Found LIBYAML: ${LIBYAML_LIBRARY}")
  set(LIBYAML_FOUND TRUE)
ELSE()
  IF(LIBYAML_FIND_REQUIRED)
    message(FATAL_ERROR "NOT FOUND: LIBYAML")
  ELSE()
    message(STATUS "NOT FOUND: LIBYAML")
  ENDIF()
endif()
  
mark_as_advanced(
        LIBYAML_INCLUDE_DIR
        LIBYAML_LIBRARY
	)
