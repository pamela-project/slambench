#######################################################
# This file is the CMAKE script for SLAMBench Libraries
#######################################################
cmake_minimum_required(VERSION 2.8)
project(slambench)
# Find most common packages
#######################################################
find_package(GLUT)
find_package(OpenGL)
find_package(Eigen3 REQUIRED)
find_package(PAPI)

#### BUILD SLAMBENCH AND TOOLS ####
add_subdirectory(shared) # SLAMBench Shared library
IF(NOT ANDROID)
    add_subdirectory(tools)  # SLAMBench Tools
ENDIF()

