#######################################################
# This file is the CMAKE script for SLAMBench Libraries
#######################################################

cmake_minimum_required(VERSION 2.8)

project(slambench)

# Most common compiler flags
#######################################################


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   -Wall -Wextra -Wno-error=unknown-pragmas")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall  -Wextra -Wno-error=unknown-pragmas")

if (CMAKE_COMPILER_IS_GNUCC)
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 6.0)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}      -Wno-error=ignored-attributes -Wno-misleading-indentation")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wno-error=ignored-attributes -Wno-misleading-indentation")
  endif()
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}     -Wno-error=implicit-fallthrough= -Wno-int-in-bool-context ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=implicit-fallthrough= -Wno-int-in-bool-context ")
  endif()
endif()

set(OPTFLAGS "-O3")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g ${OPTFLAGS}                 -Wall -Wno-unknown-pragmas -Wextra -Werror -Wno-error=deprecated-copy -Wno-error=deprecated-declarations -Wno-error=unused-result -Wno-error=reorder ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g ${OPTFLAGS} --std=c++11 -Wall -Wno-unknown-pragmas -Wextra -Werror -Wno-error=deprecated-copy -Wno-error=deprecated-declarations -Wno-error=unused-result -Wno-error=reorder ")


if(APPLE)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D__APPLE__")
  endif(APPLE)
  

# Find most common packages
#######################################################

find_package(GLUT)
find_package(OpenGL)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PAPI)




#### BUILD SLAMBENCH
#######################################################


add_subdirectory(shared) # SLAMBench Shared library


#### DEFINE GLOBAL VARIABLES (USED BY TOOLS) 
#######################################################

SET(SLAMBENCH_INCLUDE_DIR  ${EIGEN3_INCLUDE_DIR};${CMAKE_CURRENT_SOURCE_DIR}/shared/include)
SET(SLAMBENCH_LIBRARIES    stdc++ slambench-utils -Wl,--whole-archive slambench-io slambench-metrics -Wl,--no-whole-archive)


#### BUILD TOOLS
#######################################################


IF(NOT ANDROID)
    add_subdirectory(tools)  # SLAMBench Tools
ENDIF()

