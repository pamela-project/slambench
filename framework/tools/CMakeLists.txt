cmake_minimum_required(VERSION 2.8)

include_directories(${SLAMBENCH_INCLUDE_DIR})

add_subdirectory(accuracy-tools)  # SLAMBench reconstruction Error etc...
add_subdirectory(dataset-tools)  # SLAMBench Dataset Generator
add_subdirectory(profiling-tools)  # SLAMBench Profiling
add_subdirectory(filters)  # SLAMBench Filters
add_subdirectory(loaders)  # SLAMBench Loaders

