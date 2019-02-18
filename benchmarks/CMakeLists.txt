#######################################################
# This file is the CMAKE script for Benchmarks Libraries
#######################################################

cmake_minimum_required(VERSION 2.8)

project(benchmarks)


if (NOT SLAMBENCH_INCLUDE_DIR)
  message( FATAL_ERROR "SLAMBENCH_INCLUDE_DIR not found")
  else ()
  include_directories(${SLAMBENCH_INCLUDE_DIR})
endif()


## EXTERNAL SOURCE CODE, SKIP WARNINGS ....
############################################################

set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}    -Wno-error")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wno-error")




#### ADD USE-CASES DIRECTORIES ###
############################################################



if ("${APPS}" STREQUAL "all")
    message(STATUS "Select all applications...")
    set(APPS_MOD "efusion;kfusion;lsdslam;monoslam;infinitam;ptam;okvis;orbslam2;svo")
else()
    message(STATUS "Select only ${APPS} applications...")
    string(REPLACE "," ";" APPS_MOD "${APPS}")     
endif()  

set(app_list ${APPS_MOD})
  MESSAGE(STATUS "Application loop over ${APPS_MOD}")
FOREACH(app ${app_list})
    IF(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${app}")
      MESSAGE(STATUS "Load application ${app}")
      ADD_SUBDIRECTORY(${app})
    ELSE()
      MESSAGE(STATUS "Use-case ${app} not found. Please try \" make ${app} \" to download the usecase.")
    ENDIF()
ENDFOREACH(app)
