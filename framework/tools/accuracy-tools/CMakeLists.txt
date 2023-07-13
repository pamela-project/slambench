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
ENDIF(Pangolin_FOUND)

