cmake_minimum_required(VERSION 2.8)

project(slambench2_repository)


# CMake variables
#######################################################

MESSAGE(STATUS CMAKE_MODULE_PATH=${CMAKE_MODULE_PATH})

SET(EXECUTABLE_OUTPUT_PATH         ${PROJECT_BINARY_DIR}/bin CACHE PATH "Build directory" FORCE)
SET(LIBRARY_OUTPUT_PATH            ${PROJECT_BINARY_DIR}/lib CACHE PATH "Build directory" FORCE)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
SET(CMAKE_BUILD_FILES_DIRECTORY    ${PROJECT_BINARY_DIR}/bfd)
SET(CMAKE_BINARY_DIR               ${PROJECT_BINARY_DIR}/bindir)
SET(CMAKE_CACHEFILE_DIR            ${PROJECT_BINARY_DIR}/cache)
SET(CMAKE_BUILD_DIRECTORY          ${PROJECT_BINARY_DIR}/builddir)
  
  

# Most common compiler flags
#######################################################

set(GENERAL_COMPILATION_FLAGS "-g -O3  -Wall  -Wextra")


if (CMAKE_COMPILER_IS_GNUCC)
  add_definitions (-D_FILE_OFFSET_BITS=64) ## Workaround for 32 bit boards with fwrite 2G limit

  set(GENERAL_COMPILATION_FLAGS "${GENERAL_COMPILATION_FLAGS} -Wno-error=deprecated-declarations ") # Eigen raises deprecated declarations warnings and int-in-bool-context warnings.
  set(GENERAL_COMPILATION_FLAGS "${GENERAL_COMPILATION_FLAGS}  -Wno-unknown-pragmas  -Wno-error=deprecated-declarations -Wno-error=unused-result -Wno-error=reorder ")

  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 6.0)
    set(GENERAL_COMPILATION_FLAGS "${GENERAL_COMPILATION_FLAGS}      -Wno-error=ignored-attributes -Wno-misleading-indentation")
    
  endif()
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0)
    set(GENERAL_COMPILATION_FLAGS "${GENERAL_COMPILATION_FLAGS}     -Wno-error=implicit-fallthrough= -Wno-int-in-bool-context ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}    -Wno-catch-value")   # too noisy when using Pangolin
  endif()
endif()

if(APPLE)
  set(GENERAL_COMPILATION_FLAGS "${GENERAL_COMPILATION_FLAGS} -D__APPLE__")
endif(APPLE)

  
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}                ${GENERAL_COMPILATION_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  --std=c++11 ${GENERAL_COMPILATION_FLAGS}")
  
  
    
  
# DEFINE SLAMBENCH LIBS/INCLUDES
######################################################

find_package(Eigen3    REQUIRED)

SET(SLAMBENCH_INCLUDE_DIR  ${EIGEN3_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/framework/shared/include CACHE STRING "The include paths needed to use SLAMBENCH")
SET(SLAMBENCH_LIBRARIES    slambench-utils         -Wl,--whole-archive slambench-io                slambench-metrics  -Wl,--no-whole-archive)
SET(SLAMBENCH_C_WRAPPER                            -Wl,--whole-archive slambench-c-wrapper                            -Wl,--no-whole-archive)


# SLAMBENCH FUNCTIONS 
#######################################################

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

#### SUB FOLDERS
######################################################

add_subdirectory(framework)  # SLAMBench framework
add_subdirectory(benchmarks) # Benchmarks
