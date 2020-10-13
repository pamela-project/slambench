cmake_minimum_required(VERSION 2.8)

######################################################################################
########## OPENCL Wrapper for profiling  (optional) 
######################################################################################
find_package(OpenCL)

if (OpenCL_FOUND)
    include_directories(${OPENCL_INCLUDE_DIRS})
    add_library(oclwrapper SHARED OCLWrapper.cpp)
    target_link_libraries(oclwrapper ${OPENCL_LIBRARIES})
else (OpenCL_FOUND)
    message(STATUS "OpenCL wrapper cannot be build, OpenCL not found.")
endif(OpenCL_FOUND)


