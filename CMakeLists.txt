cmake_minimum_required(VERSION 2.8)

project(slambench2_repository)


# CMake variables
#######################################################

MESSAGE(STATUS CMAKE_MODULE_PATH=${CMAKE_MODULE_PATH})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Build directory" FORCE)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib CACHE PATH "Build directory" FORCE)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
SET(CMAKE_BUILD_FILES_DIRECTORY ${PROJECT_BINARY_DIR}/bfd)
SET(CMAKE_BINARY_DIR  ${PROJECT_BINARY_DIR}/bindir)
SET(CMAKE_CACHEFILE_DIR  ${PROJECT_BINARY_DIR}/cache)
SET(CMAKE_BUILD_DIRECTORY ${PROJECT_BINARY_DIR}/builddir)




add_subdirectory(framework)  # SLAMBench framework
add_subdirectory(filters)    # SLAMBench filters

add_subdirectory(benchmarks) # Benchmarks
