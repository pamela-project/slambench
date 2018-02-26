

find_path(SLAMBENCH_INCLUDE_PATH SLAMBenchAPI.h)

find_library(SLAMBENCH_UTILS_LIBRARY libslambench-utils.a PATH ${SLAMBENCH_LIBRARY_PATH})
find_library(SLAMBENCH_IO_LIBRARY libslambench-io.a PATH ${SLAMBENCH_LIBRARY_PATH})
find_library(SLAMBENCH_C_WRAPPER_LIBRARY libslambench-c-wrapper.a PATH ${SLAMBENCH_LIBRARY_PATH})
find_library(SLAMBENCH_METRICS_LIBRARY libslambench-metrics.a PATH ${SLAMBENCH_LIBRARY_PATH})

if(SLAMBENCH_INCLUDE_PATH)
  if(SLAMBENCH_UTILS_LIBRARY)
    set(SLAMBENCH_FOUND TRUE)
    SET(SLAMBENCH_INCLUDE_DIR  ${EIGEN3_INCLUDE_DIR} ${SLAMBENCH_INCLUDE_PATH} CACHE STRING "The include paths needed to use SLAMBENCH")
    SET(SLAMBENCH_LIBRARIES    ${SLAMBENCH_UTILS_LIBRARY} -Wl,--whole-archive ${SLAMBENCH_IO_LIBRARY}        ${SLAMBENCH_METRICS_LIBRARY}  -Wl,--no-whole-archive)
    SET(SLAMBENCH_C_WRAPPER                               -Wl,--whole-archive ${SLAMBENCH_C_WRAPPER_LIBRARY}                               -Wl,--no-whole-archive)
  else()
    MESSAGE(STATUS "SLAMBENCH libraries are missing.")
  endif()
else()
  MESSAGE(STATUS "SLAMBENCH headers are missing.")
endif()
mark_as_advanced(
  SLAMBENCH_INCLUDE_DIR
  SLAMBENCH_LIBRARIES
  SLAMBENCH_C_WRAPPER
  )


# Generate appropriate messages
if(SLAMBENCH_FOUND)
  if(NOT SLAMBENCH_FIND_QUIETLY)
    message("-- Found SLAMbench2: ${SLAMBENCH_INCLUDE_DIR}")
  endif(NOT SLAMBENCH_FIND_QUIETLY)
else(SLAMBENCH_FOUND)
  if(SLAMBENCH_FIND_REQUIRED)
    if(NOT SLAMBENCH_INCLUDE_PATH)
      message(FATAL_ERROR "-- Could NOT find SLAMBENCH (missing: SLAMBENCH_INCLUDE_PATH)")
    else ()
      message(FATAL_ERROR "-- Could NOT find SLAMBENCH (missing: SLAMBENCH_LIBRARY_PATH)")
    endif()
    
  endif(SLAMBENCH_FIND_REQUIRED)
endif(SLAMBENCH_FOUND)
