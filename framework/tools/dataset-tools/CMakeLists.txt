cmake_minimum_required(VERSION 2.8)

include_directories(./include)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}     -Wno-error=unused-parameter -Wno-error=unused-variable -Wno-error=unused-but-set-variable")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=unused-parameter -Wno-error=unused-variable -Wno-error=unused-but-set-variable")

# use colour to highlight error messages
string(ASCII 27 ESC)
string(ASCII 10 LF)
set(BoldRed     "${ESC}[1;31m")
set(BoldGreen   "${ESC}[1;32m")
set(ColourReset "${ESC}[m")

#------------------------------------------------
### find required external dependencies
### NOTE: dependencies vary according to dataset
#------------------------------------------------
# Boost is required - abort if not found
find_package(Boost 1.54 REQUIRED COMPONENTS system filesystem program_options regex)

# EUROCMAV datasets support requirements - skip if not found
find_package(LIBYAML QUIET)
IF (NOT LIBYAML_FOUND)
	message(STATUS "${BoldRed}LibYAML dependency for UZH-FPV, EuRoC-MAV and OpenLORIS datasets not found - datasets skipped${ColourReset}")
ENDIF ()

#------------------------------------------------
# TUM rosbag datasets support requirements - skip if not found
# OpenCV should be looked for before ROS packages in order to find
# the OpenCV version used in SLAMBench, not any in the ROS environment
find_package(OpenCV QUIET)
IF (NOT OpenCV_FOUND)
	message(STATUS "${BoldRed}OpenCV dependency for TUM rosbag datasets not found${ColourReset}")
ENDIF ()

# Find ROS distribution - if one is installed
# check if ROS environment has been setup
if (NOT "x$ENV{ROS_DISTRO}" STREQUAL "x")
	set(ROS_DIST $ENV{ROS_DISTRO})
	message (STATUS "Found ROS distribution: ${ROS_DIST}")
else ()
	# if ROS not setup, try standard location
	if (EXISTS "/opt/ros/")
		# get ROS distribution name
		execute_process(COMMAND "ls" WORKING_DIRECTORY "/opt/ros/" OUTPUT_VARIABLE SDS)
		string (STRIP ${SDS} SUBDIRS)
		string(REPLACE ${LF} ";" ROS_DISTS ${SUBDIRS})
		list (LENGTH ROS_DISTS LENSD)
		if (${LENSD} EQUAL 1)
			list (GET ROS_DISTS 0 ROS_DIST)
			list (APPEND CMAKE_PREFIX_PATH "/opt/ros/${ROS_DIST}")
			message (STATUS "Found ROS distribution: ${ROS_DIST}")
		elseif (${LENSD} GREATER 1)
			message (STATUS "${BoldRed}Found more than one ROS distribution - please setup ROS environment before building SLAMBench${ColourReset}")
		else ()
			message (STATUS "${BoldRed}Could NOT find a ROS distribution${ColourReset}")
		endif ()
	endif ()
endif ()

# Find required ROS packages
set(ROSPKGS rosbag_storage roscpp_serialization)
set(ROSPKGS_FOUND TRUE)
foreach (ROSPKG ${ROSPKGS})
	find_package(${ROSPKG} QUIET)
	if (NOT ${ROSPKG}_FOUND)
		set(ROSPKGS_FOUND FALSE)
		message(STATUS "${BoldRed}${ROSPKG} dependency for TUM rosbag datasets not found${ColourReset}")
	endif ()
endforeach (ROSPKG)

# report success or failure in finding tum-rosbag dependencies
IF (OpenCV_FOUND AND ROSPKGS_FOUND)
	message (STATUS "${BoldGreen}Generation of tum-rosbag dataset support : Activated${ColourReset}")
else ()
	message(STATUS "${BoldRed}Dependencies for TUM rosbag datasets not found - dataset skipped${ColourReset}")
endif ()
#------------------------------------------------

#------------------------------------------------
### add executables and target libraries
#------------------------------------------------
add_executable(io-readply io-readply.cpp)
target_link_libraries(io-readply slambench-io slambench-metrics)

add_executable(io-inspect-file io-inspect.cpp)
target_link_libraries(io-inspect-file PRIVATE -Wl,--whole-archive slambench-io slambench-metrics -Wl,--no-whole-archive)

IF (LIBYAML_FOUND)
	add_library(EUROCMAV EUROCMAV.cpp)
	target_link_libraries(EUROCMAV slambench-io ${Boost_LIBRARIES} ${LIBYAML_LIBRARY})
	target_include_directories(EUROCMAV PRIVATE ${Boost_INCLUDE_DIRS} ${LIBYAML_INCLUDE_DIR})
	set(EUROCMAV_LIB "EUROCMAV")

	add_library(OpenLORIS OpenLORIS.cpp)
	target_link_libraries(OpenLORIS slambench-io ${Boost_LIBRARIES} ${LIBYAML_LIBRARY})
	target_include_directories(OpenLORIS PRIVATE ${Boost_INCLUDE_DIRS} ${LIBYAML_INCLUDE_DIR})
	set(OpenLORIS_LIB "OpenLORIS")

	add_library(UZHFPV UZHFPV.cpp)
	target_link_libraries(UZHFPV slambench-io ${Boost_LIBRARIES} ${LIBYAML_LIBRARY})
	target_include_directories(UZHFPV PRIVATE ${Boost_INCLUDE_DIRS} ${LIBYAML_INCLUDE_DIR})
	set(UZHFPV_LIB "UZHFPV")

ENDIF (LIBYAML_FOUND)

add_library(SVO SVO.cpp)
target_link_libraries(SVO slambench-io ${Boost_LIBRARIES})
target_include_directories(SVO PRIVATE ${Boost_INCLUDE_DIRS})

add_library(ICL ICL.cpp)
target_link_libraries(ICL slambench-io ${Boost_LIBRARIES})
target_include_directories(ICL PRIVATE ${Boost_INCLUDE_DIRS})

add_library(TUM TUM.cpp)
target_link_libraries(TUM slambench-io ${Boost_LIBRARIES})
target_include_directories(TUM PRIVATE ${Boost_INCLUDE_DIRS})

add_library(BONN BONN.cpp)
target_link_libraries(BONN slambench-io ${Boost_LIBRARIES})
target_include_directories(BONN PRIVATE ${Boost_INCLUDE_DIRS})

add_library(ICLNUIM ICLNUIM.cpp)
target_link_libraries(ICLNUIM slambench-io)
target_include_directories(ICLNUIM PRIVATE ${Boost_INCLUDE_DIRS})

add_library(ETHI ETHI.cpp)
target_link_libraries(ETHI slambench-io TUM ICLNUIM)
target_include_directories(ETHI PRIVATE ${Boost_INCLUDE_DIRS})

add_library(VolumeDeform VolumeDeform.cpp)
target_link_libraries(VolumeDeform slambench-io)
target_include_directories(VolumeDeform PRIVATE ${Boost_INCLUDE_DIRS})

find_package(PCL QUIET)

IF (OpenCV_FOUND AND PCL_FOUND)
	add_library(KITTI KITTI.cpp)
	target_link_libraries(KITTI slambench-io ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
	target_include_directories(KITTI PRIVATE ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
endif ()

IF (PCL_FOUND)
	add_library(NSH NSH.cpp)
	target_link_libraries(NSH slambench-io ${PCL_LIBRARIES})
	target_include_directories(NSH PRIVATE ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
endif ()

IF (PCL_FOUND)
	add_library(NewerCollege NewerCollege.cpp)
	target_link_libraries(NewerCollege slambench-io ${PCL_LIBRARIES})
	target_include_directories(NewerCollege PRIVATE ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
endif ()

IF (OpenCV_FOUND AND PCL_FOUND)
	add_library(DARPASubt DARPASubt.cpp)
	target_link_libraries(DARPASubt slambench-io ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
	target_include_directories(DARPASubt PRIVATE ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
elseif(PCL_FOUND)
	add_library(DARPASubt DARPASubt.cpp)
	target_link_libraries(DARPASubt slambench-io ${PCL_LIBRARIES})
	target_include_directories(DARPASubt PRIVATE ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
endif ()

# if all dependencies found add TUM-ROSBAG target
IF (OpenCV_FOUND AND ROSPKGS_FOUND)
	add_library(TUM-ROSBAG TUM-ROSBAG.cpp)
	# in some platforms, newer versions of the gnu linker
	# changed the search PATHs for dependency libraries.
	# This option uses old-style PATHs.
	target_link_libraries(TUM-ROSBAG "-Wl,--disable-new-dtags")
	target_link_libraries(TUM-ROSBAG slambench-io ${Boost_LIBRARIES})
	target_link_libraries(TUM-ROSBAG ${OpenCV_LIBRARIES})
	foreach (ROSPKG ${ROSPKGS})
		target_link_libraries(TUM-ROSBAG ${${ROSPKG}_LIBRARIES})
	endforeach (ROSPKG)
	target_include_directories(TUM-ROSBAG PRIVATE ${Boost_INCLUDE_DIRS})
	target_include_directories(TUM-ROSBAG PRIVATE ${OpenCV_INCLUDE_DIRS})
	target_include_directories(TUM-ROSBAG PRIVATE ${rosbag_storage_INCLUDE_DIRS})
	add_definitions(-DROSBAG_SUPPORT=1)
	set(TUM-ROSBAG_LIB "TUM-ROSBAG")
endif ()

add_executable(dataset-generator dataset-generator.cpp)
target_link_libraries(dataset-generator KITTI NSH NewerCollege DARPASubt TUM UZHFPV ICLNUIM ICL ETHI BONN VolumeDeform ${OpenLORIS_LIB} ${UZHFPV_LIB} ${EUROCMAV_LIB} SVO ${TUM-ROSBAG_LIB})
