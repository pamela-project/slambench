cmake_minimum_required(VERSION 2.8)

include_directories(./include)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}     -Wno-error=unused-parameter -Wno-error=unused-variable -Wno-error=unused-but-set-variable")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=unused-parameter -Wno-error=unused-variable -Wno-error=unused-but-set-variable")

# use colour to highlight error messages
string(ASCII 27 ESC)
string(ASCII 10 LF)
set(BoldRed     "${ESC}[1;31m")
set(ColourReset "${ESC}[m")

# Boost is required - abort if not found
find_package(Boost 1.54 REQUIRED COMPONENTS system filesystem program_options regex)

add_executable(io-readply io-readply.cpp)
target_link_libraries(io-readply slambench-io)

add_executable(io-inspect-file io-inspect.cpp)
target_link_libraries(io-inspect-file PRIVATE -Wl,--whole-archive slambench-io -Wl,--no-whole-archive)

# EUROCMAV datasets support requirements - skip if not found
find_package(LIBYAML QUIET)
IF (LIBYAML_FOUND)
	add_library(EUROCMAV EUROCMAV.cpp)
	target_link_libraries(EUROCMAV slambench-io ${Boost_LIBRARIES} ${LIBYAML_LIBRARY})
	target_include_directories(EUROCMAV PRIVATE ${Boost_INCLUDE_DIRS}  ${LIBYAML_INCLUDE_DIR})
	set(EUROCMAV_LIB "EUROCMAV")
ELSE ()
	message(STATUS "${BoldRed}LibYAML dependency for EUROCMAV datasets not found - dataset skipped${ColourReset}")
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

# TUM rosbag datasets support requirements - skip if not found
# OpenCV should be looked for before ROS packages in order to find
# the OpenCV version used in SLAMBench, not any in the ROS environment
find_package(OpenCV QUIET)
IF (NOT OpenCV_FOUND)
	message(STATUS "${BoldRed}OpenCV dependency for TUM rosbag datasets not found${ColourReset}")
ENDIF ()

# Find ROS distribution
# check if ROS environment has been setup
if ( "x$ENV{ROS_DISTRO}" STREQUAL "x")
	# if not setup try standard place
	if (EXISTS "/opt/ros/")
		# get ROS distribution name
		execute_process(COMMAND "ls" WORKING_DIRECTORY "/opt/ros/" OUTPUT_VARIABLE SDS)
		string (STRIP ${SDS} SUBDIRS)
		string(REPLACE ${LF} ";" ROSDISTS ${SUBDIRS})
		list (LENGTH ROSDISTS LENSD)
		if (${LENSD} EQUAL 1)
			list (GET ROSDISTS 0 ROSDIST)
			list (APPEND CMAKE_PREFIX_PATH "/opt/ros/${ROSDIST}")
			message (STATUS "Found ROS distribution: ${ROSDIST}")
		elseif (${LENSD} GREATER 1)
			message (STATUS "${BoldRed}Found more than one ROS distribution - please setup ROS environment before building SLAMBench${ColourReset}")
		else ()
			message (STATUS "${BoldRed}Could NOT find a ROS distribution${ColourReset}")
		endif ()
	endif ()
else ()
	message (STATUS "Found ROS distribution: $ENV{ROS_DISTRO}")
endif ()

# Find required ROS packages
foreach (ROSPKG cpp_common roscpp_serialization rostime rosbag_storage)
	find_package(${ROSPKG} QUIET)
	if (NOT ${ROSPKG}_FOUND)
		message(STATUS "${BoldRed}${ROSPKG} dependency for TUM rosbag datasets not found${ColourReset}")
	endif ()
endforeach (ROSPKG)

# ROS package tf is treated differently to avoid a link library conflict
# check for required tf include files only - no link libraries required
# ROS packages have common include directories so use any of the above
if (cpp_common_FOUND)
	foreach (INCDIR ${cpp_common_INCLUDE_DIRS})
		if (EXISTS "${INCDIR}/tf")
			set(tf_FOUND TRUE)
		endif ()
	endforeach()
endif ()
if (NOT tf_FOUND)
	message(STATUS "${BoldRed}tf dependency for TUM rosbag datasets not found${ColourReset}")
endif ()

# if all dependencies found add TUM-ROSBAG target
IF (OpenCV_FOUND AND cpp_common_FOUND AND roscpp_serialization_FOUND
		AND rostime_FOUND AND rosbag_storage_FOUND AND tf_FOUND)
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
	message(STATUS "${BoldRed}ROS dependencies for TUM rosbag datasets not found - dataset skipped${ColourReset}")
endif ()

add_library(ICLNUIM ICLNUIM.cpp)
target_link_libraries(ICLNUIM slambench-io)
target_include_directories(ICLNUIM PRIVATE ${Boost_INCLUDE_DIRS})

add_executable(dataset-generator dataset-generator.cpp)
target_link_libraries(dataset-generator TUM ICLNUIM ICL ${EUROCMAV_LIB} SVO ${TUM-ROSBAG_LIB})
