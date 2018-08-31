cmake_minimum_required(VERSION 2.8)



find_package(PCL  1.5)
## FIXME: PCL by using UseVTK.cmake corrupts COMPILE_DEFINITIONS
get_directory_property(DirDefs  COMPILE_DEFINITIONS )
message(STATUS "VTK FIX:Previous properties=${DirDefs}")
set_directory_properties(PROPERTIES COMPILE_DEFINITIONS "")


######################################################################################
#################        COMMONS
######################################################################################


include_directories(. ${PROJECT_SOURCE_DIR}/shared/include)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}     -Wno-error=unused-parameter -Wno-error=unused-variable -Wno-error=unused-but-set-variable")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=unused-parameter -Wno-error=unused-variable -Wno-error=unused-but-set-variable")


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
	
	target_link_libraries(pointcloud_aligner PRIVATE ${gui_pangolin_libraries}  ${PCL_LIBRARIES})
    SET_TARGET_PROPERTIES(pointcloud_aligner PROPERTIES COMPILE_FLAGS "-Wno-error=catch-value=  -Wno-error=comment")
 
    install(TARGETS pointcloud_aligner DESTINATION  bin/)

   
else (PCL_FOUND)
         message(STATUS "Trajectory tool will not be compile, PCL not found.")
endif(PCL_FOUND)

	
ENDIF(Pangolin_FOUND)

