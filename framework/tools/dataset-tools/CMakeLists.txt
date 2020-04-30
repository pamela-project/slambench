cmake_minimum_required(VERSION 2.8)

include_directories(./include)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}     -Wno-error=unused-parameter -Wno-error=unused-variable -Wno-error=unused-but-set-variable")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=unused-parameter -Wno-error=unused-variable -Wno-error=unused-but-set-variable")

find_package(Boost 1.54 REQUIRED COMPONENTS system filesystem program_options regex)
IF (NOT Boost_FOUND)
	message(FATAL_ERROR "Dataset generators requirement not found (Boost) - Abort.")
	return()
endif ()

# EUROCMAV datasets support requirements
find_package(LIBYAML REQUIRED)

# TUM rosbag datasets support requirements
# OpenCV should be looked for before catkin to find
# the version used in SLAMBench, not the one in the ROS environment
find_package(OpenCV REQUIRED)
find_package(cpp_common QUIET)
find_package(roscpp_serialization QUIET)
find_package(rostime QUIET)
find_package(rosbag_storage QUIET)

add_executable(io-readply io-readply.cpp)
target_link_libraries(io-readply slambench-io)

add_executable(io-inspect-file io-inspect.cpp)
target_link_libraries(io-inspect-file PRIVATE -Wl,--whole-archive slambench-io -Wl,--no-whole-archive)

IF (LIBYAML_FOUND)
	add_library(EUROCMAV EUROCMAV.cpp)
	target_link_libraries(EUROCMAV slambench-io ${Boost_LIBRARIES} ${LIBYAML_LIBRARY})
	target_include_directories(EUROCMAV PRIVATE ${Boost_INCLUDE_DIRS}  ${LIBYAML_INCLUDE_DIR})
ELSE ()
	message(WARNING "EUROCMAV datasets support not found (LibYAML) - Skipped.")
ENDIF ()

add_library(SVO SVO.cpp)
target_link_libraries(SVO slambench-io ${Boost_LIBRARIES})
target_include_directories(SVO PRIVATE ${Boost_INCLUDE_DIRS})

add_library(ICL ICL.cpp)
target_link_libraries(ICL slambench-io ${Boost_LIBRARIES})
target_include_directories(ICL PRIVATE ${Boost_INCLUDE_DIRS})

add_library(TUM TUM.cpp)
target_link_libraries(TUM slambench-io ${Boost_LIBRARIES})
target_include_directories(TUM PRIVATE ${Boost_INCLUDE_DIRS})

IF (cpp_common_FOUND AND roscpp_serialization_FOUND AND rostime_FOUND
		AND rosbag_storage_FOUND)
	add_library(TUM-ROSBAG TUM-ROSBAG.cpp)
	target_include_directories(TUM-ROSBAG PRIVATE ${Boost_INCLUDE_DIRS})
	target_include_directories(TUM-ROSBAG PRIVATE ${cpp_common_INCLUDE_DIRS})
	target_include_directories(TUM-ROSBAG PRIVATE ${roscpp_serialization_INCLUDE_DIRS})
	target_include_directories(TUM-ROSBAG PRIVATE ${rostime_INCLUDE_DIRS})
	target_include_directories(TUM-ROSBAG PRIVATE ${rosbag_storage_INCLUDE_DIRS})
	target_link_libraries(TUM-ROSBAG ${OpenCV_LIBS})
	target_link_libraries(TUM-ROSBAG ${cpp_common_LIBRARIES})
	target_link_libraries(TUM-ROSBAG ${roscpp_serialization_LIBRARIES})
	target_link_libraries(TUM-ROSBAG ${rostime_LIBRARIES})
	target_link_libraries(TUM-ROSBAG ${rosbag_storage_LIBRARIES})
	target_link_libraries(TUM-ROSBAG slambench-io ${Boost_LIBRARIES})
	add_definitions(-DROSBAG_SUPPORT=1)
	set(TUM-ROSBAG_LIB "TUM-ROSBAG")
else ()
	message(STATUS "TUM rosbag support not found - skipped.")
endif ()

add_library(ICLNUIM ICLNUIM.cpp)
target_link_libraries(ICLNUIM slambench-io)
target_include_directories(ICLNUIM PRIVATE ${Boost_INCLUDE_DIRS})

add_executable(dataset-generator dataset-generator.cpp)
target_link_libraries(dataset-generator TUM ICLNUIM ICL EUROCMAV SVO ${TUM-ROSBAG_LIB})
