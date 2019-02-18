cmake_minimum_required(VERSION 2.8)



######################################################################################
#################        COMMONS
######################################################################################

include_directories(. ${PROJECT_SOURCE_DIR}/shared/include)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}     -Wno-error=unused-parameter -Wno-error=unused-variable -Wno-error=unused-but-set-variable")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=unused-parameter -Wno-error=unused-variable -Wno-error=unused-but-set-variable")


if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 8.0)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=catch-value=")
endif()
	
######################################################################################
################# DYNAMIC LIBRARY SYSTEM
######################################################################################

add_executable(benchmark_loader benchmark_loader.cpp)
TARGET_INCLUDE_DIRECTORIES(benchmark_loader PUBLIC ${SLAMBENCH_INCLUDE_DIR})
target_link_libraries(benchmark_loader PRIVATE pthread ${SLAMBENCH_LIBRARIES})

find_package(Pangolin QUIET)
find_package(TOON QUIET)

IF(Pangolin_FOUND)
	set(main_common_libraries     ${SLAMBENCH_LIBRARIES})
	set(gui_pangolin_libraries     ${main_common_libraries} slambench-ui-pangolin ${Pangolin_LIBRARIES} ${OPENGL_LIBRARIES} pthread)
	add_executable(pangolin_loader pangolin_loader.cpp)
	TARGET_INCLUDE_DIRECTORIES(pangolin_loader PUBLIC ${SLAMBENCH_INCLUDE_DIR})
	TARGET_INCLUDE_DIRECTORIES(pangolin_loader PUBLIC ${Pangolin_INCLUDE_DIR})
	TARGET_INCLUDE_DIRECTORIES(pangolin_loader PUBLIC ${TOON_INCLUDE_PATHS})
	target_link_libraries(pangolin_loader PRIVATE ${gui_pangolin_libraries})



ENDIF(Pangolin_FOUND)


