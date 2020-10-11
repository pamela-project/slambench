cmake_minimum_required(VERSION 2.8)
project(accuracy_tools)
find_package(PCL REQUIRED)
get_directory_property(DirDefs COMPILE_DEFINITIONS)
message(STATUS "VTK FIX:Previous properties=${DirDefs}")
set_directory_properties(PROPERTIES COMPILE_DEFINITIONS "")
######################################################################################
#################        COMMONS
######################################################################################

include_directories(. ../../shared/include)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  --std=c++14   -Wno-error=unused-parameter -Wno-error=unused-variable -Wno-error=unused-but-set-variable -Wno-error=attributes")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++14 -Wno-error=unused-parameter -Wno-error=unused-variable -Wno-error=unused-but-set-variable -Wno-error=attributes")


find_package(Pangolin QUIET)
IF(Pangolin_FOUND)
	set(main_common_libraries     ${SLAMBENCH_LIBRARIES})
	set(gui_pangolin_libraries     ${main_common_libraries} slambench-ui-pangolin ${Pangolin_LIBRARIES} ${OPENGL_LIBRARIES} pthread)
if (PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})

	add_executable(pointcloud_aligner pointcloud_aligner.cpp)
	
	TARGET_INCLUDE_DIRECTORIES(pointcloud_aligner PUBLIC ${SLAMBENCH_INCLUDE_DIR})
	TARGET_INCLUDE_DIRECTORIES(pointcloud_aligner PUBLIC ${Pangolin_INCLUDE_DIR})
	TARGET_INCLUDE_DIRECTORIES(pointcloud_aligner PUBLIC ${TOON_INCLUDE_PATHS})
	
	target_link_libraries(pointcloud_aligner  PRIVATE ${gui_pangolin_libraries}  ${PCL_LIBRARIES})
	target_compile_options(pointcloud_aligner PRIVATE  "-Wno-error=comment")
 	if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 8.0)
	  target_compile_options(pointcloud_aligner     PRIVATE "-Wno-error=catch-value=")
	endif()


    install(TARGETS pointcloud_aligner DESTINATION  bin/)

   
else (PCL_FOUND)
         message(STATUS "Trajectory tool will not be compile, PCL not found.")
endif(PCL_FOUND)

	
ENDIF(Pangolin_FOUND)

