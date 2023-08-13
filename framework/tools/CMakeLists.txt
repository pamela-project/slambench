cmake_minimum_required(VERSION 3.10)

include_directories(${SLAMBENCH_INCLUDE_DIR})

# should cut out the accuracy tools 
add_subdirectory(accuracy-tools)  # SLAMBench reconstruction Error etc...
add_subdirectory(dataset-tools)  # SLAMBench Dataset Generator
add_subdirectory(profiling-tools)  # SLAMBench Profiling
add_subdirectory(loaders)  # SLAMBench Loaders

