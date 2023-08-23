cmake_minimum_required(VERSION 2.8.3)
project(legoloam)

# Use LeGO-LOAM as an example

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")

# =========================================================================================
# Instead of using find_package(), declare variables for path to 'lib' and 'include' directories,
# and specify it explicitly using cmake -D[variable_name]

# For example, Flann requires ${FLANN_LIBRARY_DIR} and ${FLANN_INCLUDE_DIR}
# then, ${FLANN_LIBRARIES} will be the lib files inside ${FLANN_LIBRARY_DIR}.

file(GLOB FLANN_LIBRARIES "${FLANN_LIBRARY_DIR}/libflann*.*")
file(GLOB PCL_LIBRARIES "${PCL_LIBRARY_DIR}/libpcl_*.so*")
file(GLOB OpenCV_LIBRARIES "${OpenCV_LIBRARY_DIR}/libopencv_*.so*")
file(GLOB GTSAM_LIBRARIES "${GTSAM_LIBRARY_DIR}/lib*.*")
file(GLOB YAML_CPP_LIBRARIES "${YAML_LIBRARY_DIR}/libyaml-cpp.a")

include_directories(
	include
    ${EIGEN3_INCLUDE_DIR}
	${FLANN_INCLUDE_DIR}
	${PCL_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIR}
	${GTSAM_INCLUDE_DIR}
	${YAML_CPP_INCLUDE_DIR}
	${SLAMBENCH_INCLUDE_DIR}
)
# =========================================================================================


# =========================================================================================
# Create library for algorithm source code
add_library(${PROJECT_NAME} SHARED
./include/cloud_info.h
./include/utility.h
./include/imageProjection.h
./include/featureAssociation.h
./include/mapOptimization.h
./include/legoloam.h
./src/imageProjection.cpp
./src/featureAssociation.cpp
./src/mapOptimization.cpp
./src/legoloam.cpp)
# =========================================================================================


# =========================================================================================
# Create library for legoloam_library.cpp
add_library(${PROJECT_NAME}-original SHARED legoloam_library.cpp)
# Specify the install path, so if you have: cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=$ALG_DIR/"
# liblegoloam-original.so will be installed into $ALG_DIR/lib/
install(TARGETS ${PROJECT_NAME}  DESTINATION  lib/)
install(TARGETS ${PROJECT_NAME}-original  DESTINATION  lib/)

target_link_libraries(${PROJECT_NAME}-original
	${PROJECT_NAME}
	${FLANN_LIBRARIES}
	${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
	${YAML_CPP_LIBRARIES}
    ${GTSAM_LIBRARIES})
# =========================================================================================


# =========================================================================================
SET(SLAMBENCH_C_WRAPPER	-Wl,--whole-archive -L${WRAPPER_PATH} slambench-c-wrapper	-Wl,--no-whole-archive)
SET(NULL_FILE_PATH	${WRAPPER_PATH}/null.cpp)

function(generate_slam_library library_name )

    add_library(${library_name}-library SHARED ${NULL_FILE_PATH})
    target_link_libraries(${library_name}-library PRIVATE ${SLAMBENCH_C_WRAPPER} ${library_name})
    SET_TARGET_PROPERTIES(${library_name}-library PROPERTIES LINK_FLAGS "${ARGN}")
    # ${LIB_PATH} specify the path to library directory of dependencies
    # therefore, this path is embedded into ${library_name}-library, liblegoloam-original-library.so in this case.
    SET_TARGET_PROPERTIES(${library_name}-library PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE INSTALL_RPATH "${LIB_PATH}")
    install(TARGETS ${library_name}-library  DESTINATION  lib/)

endfunction()

generate_slam_library(${PROJECT_NAME}-original "-Wl,--disable-new-dtags")
# =========================================================================================
