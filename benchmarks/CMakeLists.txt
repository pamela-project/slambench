#######################################################
# This file is the CMAKE script for Benchmarks Libraries
#######################################################

cmake_minimum_required(VERSION 2.8)

project(benchmarks)


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

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g ${OPTFLAGS}                 -Wall -Wno-unknown-pragmas -Wextra  -Wno-error=deprecated-declarations -Wno-error=unused-result -Wno-error=reorder ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g ${OPTFLAGS} --std=c++11 -Wall -Wno-unknown-pragmas -Wextra  -Wno-error=deprecated-declarations -Wno-error=unused-result -Wno-error=reorder ")


if(APPLE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D__APPLE__")
endif(APPLE)

  
######################################################
######### SELECT THE INTEFACES TO COMPILE 
######################################################

find_package(Eigen3    REQUIRED)

SET(SLAMBENCH_INCLUDE_DIR  ${EIGEN3_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/framework/shared/include CACHE STRING "The include paths needed to use SLAMBENCH")
SET(SLAMBENCH_LIBRARIES    slambench-utils         -Wl,--whole-archive slambench-io                slambench-metrics  -Wl,--no-whole-archive)
SET(SLAMBENCH_C_WRAPPER                            -Wl,--whole-archive slambench-c-wrapper                            -Wl,--no-whole-archive)
 
 
include_directories(${SLAMBENCH_INCLUDE_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=deprecated-declarations ") # Eigen raises deprecated declarations warnings and int-in-bool-context warnings.


######### SLAMBENCH FUNCTIONS ############

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColorReset  "${Esc}[m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
endif()

 SET(NULL_FILE_PATH ${PROJECT_BINARY_DIR}/null.cpp)
if (NOT EXISTS ${NULL_FILE_PATH})
 file(WRITE ${NULL_FILE_PATH} "")
 ENDIF()

function(explore_implementations algorithm_name implementations_path)

  
  set(appname ${algorithm_name})
  
  FILE(GLOB VERSIONS ${implementations_path})
  
  FOREACH(version_dir ${VERSIONS})
    IF(IS_DIRECTORY ${version_dir})
      get_filename_component(appversion ${version_dir} NAME)
      MESSAGE(STATUS "Load version ${appversion} of ${appname}.")
      ADD_SUBDIRECTORY(${version_dir})
    ENDIF()
  ENDFOREACH(version_dir)

endfunction()  
  
function(generate_slam_binaries_only library_name )
  
  message(STATUS "Potential interfaces are ${SLAMBENCH_INTERFACES}...") 
  FOREACH(INTERFACE ${SLAMBENCH_INTERFACES})
     message(STATUS "${Green}Generation of ${library_name}-${INTERFACE} : Activated${ColorReset}")      
  	
     add_executable(${library_name}-${INTERFACE}  ${NULL_FILE_PATH})
     target_link_libraries(${library_name}-${INTERFACE} PRIVATE ${library_name} ${INTERFACE} ${library_name})
     SET_TARGET_PROPERTIES(${library_name}-${INTERFACE} PROPERTIES LINK_FLAGS "${ARGN}")
     install(TARGETS ${library_name}-${INTERFACE}  DESTINATION  bin/)
  ENDFOREACH()

endfunction()

function(generate_slam_library library_name )
  
  message(STATUS "Potential interfaces are ${SLAMBENCH_INTERFACES}...") 
  FOREACH(INTERFACE ${SLAMBENCH_INTERFACES})
     message(STATUS "${Green}Generation of ${library_name}-${INTERFACE} : Activated${ColorReset}")      
  	
     add_executable(${library_name}-${INTERFACE}  ${NULL_FILE_PATH})
     target_link_libraries(${library_name}-${INTERFACE} PRIVATE ${library_name} ${INTERFACE} ${library_name})
     SET_TARGET_PROPERTIES(${library_name}-${INTERFACE} PROPERTIES LINK_FLAGS "${ARGN}")
     install(TARGETS ${library_name}-${INTERFACE}  DESTINATION  bin/)
  ENDFOREACH()
  
  message(STATUS "${Green}Generation of ${library_name}-library : Activated${ColorReset}")            
  add_library(${library_name}-library SHARED ${NULL_FILE_PATH})
  target_link_libraries(${library_name}-library PRIVATE ${SLAMBENCH_C_WRAPPER} ${library_name})
  SET_TARGET_PROPERTIES(${library_name}-library PROPERTIES LINK_FLAGS "${ARGN}")
  install(TARGETS ${library_name}-library  DESTINATION  lib/)
     
     
endfunction()


#### ADD USE-CASES DIRECTORIES ###



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
    IF(IS_DIRECTORY ${version_dir})
      MESSAGE(STATUS "Load application ${app}")
      ADD_SUBDIRECTORY(${app})
    ELSE()
      MESSAGE(STATUS "Use-case ${app} not found. Please try \" make ${app} \" to download the usecase.")
    ENDIF()
ENDFOREACH(app)
