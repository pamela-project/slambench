FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

# !!! IMPORTANT !!! Declare a variable that represents root directory of the volume
ENV ALG_DIR=/deps/algorithm-name

# Install dependencies with apt-get
RUN apt-get -y update && apt-get -y upgrade
RUN apt-get -y install git
RUN apt-get -y install g++
RUN apt-get -y install wget
RUN apt-get -y install unzip
RUN apt-get -y install cmake
# Boost is commonly used
RUN apt-get -y install libboost-all-dev
# Required by Suitesparse
RUN apt-get -y install gfortran libblas-dev liblapack-dev
# Required by Ceres
RUN apt-get -y install libgoogle-glog-dev


# /repos/ is directory for source code
RUN mkdir repos


# ========== <Eigen3.2> ==========
WORKDIR /repos
RUN git clone --depth 1 --branch 3.2.10 https://gitlab.com/libeigen/eigen.git eigen3

# Build eigen
## "-DCMAKE_INSTALL_PREFIX:PATH=$ALG_DIR/eigen3/" specifies the install path,
## after 'make install' it will contains include and lib directories, which is part of the algorithm volume.
## Note: for eigen3, there is no lib directory.
WORKDIR eigen3
RUN mkdir build
WORKDIR build
RUN cmake .. -DBUILD_SHARED_LIBS=OFF  -DCMAKE_BUILD_TYPE=Release  "-DCMAKE_INSTALL_PREFIX:PATH=$ALG_DIR/eigen3/"
RUN make -j$(expr $(nproc) - 2) && make install


# ========== <Suitesparse v4.3.1> ==========
WORKDIR /repos
RUN git clone https://github.com/jluttine/suitesparse.git suitesparse
WORKDIR suitesparse
RUN git checkout v4.3.1

# Build suitesparse
RUN make
RUN mkdir $ALG_DIR/suitesparse/include -p
RUN mkdir $ALG_DIR/suitesparse/lib -p
RUN make install INSTALL_LIB=$ALG_DIR/suitesparse/lib INSTALL_INCLUDE=$ALG_DIR/suitesparse/include


# ========== <Ceres> ==========
# Requires: <Eigen> and <Suitesparse>
WORKDIR /repos
RUN git clone "https://ceres-solver.googlesource.com/ceres-solver" ceres
WORKDIR ceres
RUN git checkout 1.12.0

# Build ceres
## we need to specify the include and lib path of eigen3 and suitesparse.
RUN mkdir build
WORKDIR build
RUN cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=$ALG_DIR/ceres" \
             "-DBUILD_EXAMPLES:BOOL=OFF" \
             "-DBUILD_TESTING:BOOL=OFF" \
             "-DBUILD_DOCUMENTATION=OFF" \
             "-DBUILD_SHARED_LIBS=ON" \
             "-DCERES_USING_SHARED_LIBRARY=OFF" \
             -DCMAKE_CXX_FLAGS="-w -O3 -std=c++11" \
             -DEIGEN_INCLUDE_DIR=$ALG_DIR/eigen3/include/eigen3 \
             -DSUITESPARSE_INCLUDE_DIR_HINTS=$ALG_DIR/suitesparse/include \
             -DSUITESPARSE_LIBRARY_DIR_HINTS=$ALG_DIR/suitesparse/lib \
             -DCXSPARSE_INCLUDE_DIR=$ALG_DIR/suitesparse/include \
             -DCXSPARSE_LIBRARY=$ALG_DIR/suitesparse/lib/libcxsparse.a
RUN make -j$(expr $(nproc) - 2) && make install


# ========== <SLAMBench> ==========
## clone a slambench as we need libslambench-c-wrapper.a
WORKDIR /repos
RUN git clone --branch fake_slambench https://github.com/nikolaradulov/slambench.git slambench

## move libslambench-c-wrapper.a and null.cpp to /deps/algorithm-name/wrapper/
WORKDIR $ALG_DIR
RUN mkdir wrapper
WORKDIR /repos/slambench/docker/wrapper
RUN mv libslambench-c-wrapper.a $ALG_DIR/wrapper
RUN mv null.cpp $ALG_DIR/wrapper


# ========== <Algorithm> ==========
WORKDIR /repos
RUN git clone --recursive https://github.com/username/algorithm.git algorithm-name

## "-DCMAKE_INSTALL_PREFIX:PATH=$ALG_DIR/"
### depends on how you write the CMakeLists.txt of the algorithm
### if you have 'install(TARGETS algorithm-library  DESTINATION  lib/)',
### then 'libalgorithm-library.so' will be install at $ALG_DIR/lib/

## "-DLIB_PATH="$ALG_DIR/lib:$ALG_DIR/ceres/lib"
### specify the path to 'lib' directory of dependencies, so generated library knows where to find the linked libraries.

### In CMakeList.txt, you would need:
### SET_TARGET_PROPERTIES(${library_name}-library PROPERTIES LINK_FLAGS "-Wl,--disable-new-dtags")
### SET_TARGET_PROPERTIES(${library_name}-library PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE INSTALL_RPATH "${LIB_PATH}")

WORKDIR algorithm-name
RUN mkdir build
WORKDIR build
RUN cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=$ALG_DIR/" \
 	-DWRAPPER_PATH=$ALG_DIR/wrapper/ \
   -DSLAMBENCH_INCLUDE_DIR=/repos/slambench/framework/shared/include/ \
 	-DEIGEN3_INCLUDE_DIR=$ALG_DIR/eigen3/include/eigen3/ \
   -DCERES_INCLUDE_DIR=$ALG_DIR/ceres/include/ \
   -DCERES_LIBRARY_DIR=$ALG_DIR/ceres/lib/ \
 	-DLIB_PATH="$ALG_DIR/lib:$ALG_DIR/ceres/lib"
RUN make -j$(expr $(nproc) - 2) && make install


# ========================== OTHER COMMONLY USED LIBRARIES ==========================
# <Eigen3>, <Suitespare>, <Ceres> See above.
# <Flann>, <TooN>, <PCL>, <OpenCV>, <Pangolin>, <G2O>, <GTSAM>, <YAML-CPP>

# You may need to change the version of libraries for your own algorithm.


# ========== <Flann> ==========
WORKDIR /repos
RUN git clone https://github.com/mariusmuja/flann.git flann
WORKDIR flann
RUN git checkout 06a49513138009d19a1f4e0ace67fbff13270c69
RUN touch /repos/flann/src/cpp/empty.cpp
RUN sed -e "/add_library(flann_cpp SHARED/ s/\"\"/empty.cpp/" -e "/add_library(flann SHARED/ s/\"\"/empty.cpp/" -i /repos/flann/src/cpp/CMakeLists.txt

# build flann
RUN mkdir build
WORKDIR build
RUN cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=$ALG_DIR/flann/" \
            -DBUILD_MATLAB_BINDINGS=FALSE \
            -DBUILD_PYTHON_BINDINGS=FALSE \
            -DBUILD_EXAMPLES=FALSE \
            -DBUILD_TESTS=FALSE \
            -DBUILD_DOC=FALSE
RUN make -j$(expr $(nproc) - 2) && make install



# ========== <TooN> ==========
WORKDIR /repos
RUN git clone "https://github.com/edrosten/TooN.git" TooN
WORKDIR TooN
RUN git checkout 92241416d2a4874fd2334e08a5d417dfea6a1a3f

# Build TooN
RUN ./configure --prefix=$ALG_DIR/toon/
RUN make install



# ========== <PCL 1.8> ==========
# Requires: <Flann>
RUN apt-get -y install libboost-all-dev freeglut3 freeglut3-dev libglew-dev libglu1-mesa libglu1-mesa-dev libgl1-mesa-glx libgl1-mesa-dev
WORKDIR /repos
RUN git clone https://github.com/PointCloudLibrary/pcl.git pcl
WORKDIR pcl
RUN git checkout 6fb1b65d3099a915255b070269b1ac78ed384921
RUN sed -i.bak "s/\(find_package.*\) mpi/\1/" /repos/pcl/cmake/pcl_find_boost.cmake

# build pcl
RUN mkdir build
WORKDIR build
RUN cmake .. -DWITH_LIBUSB=FALSE \
	-DWITH_OPENNI=FALSE \
	-DWITH_OPENNI2=FALSE \
	-DWITH_FZAPI=FALSE \
	-DWITH_ENSENSO=FALSE \
	-DWITH_DAVIDSDK=FALSE \
	-DWITH_DSSDK=FALSE \
	-DWITH_RSSDK=FALSE \
	-DWITH_VTK=FALSE \
	-DWITH_PCAP=FALSE \
	-DBUILD_common=TRUE \
	-DBUILD_octree=TRUE \
	-DBUILD_io=TRUE \
	-DBUILD_kdtree=TRUE \
	-DBUILD_search=TRUE \
	-DBUILD_sample_consensus=TRUE \
	-DBUILD_filters=TRUE \
	-DBUILD_2d=TRUE \
	-DBUILD_geometry=TRUE \
	-DBUILD_features=TRUE \
	-DBUILD_ml=TRUE \
	-DBUILD_segmentation=FALSE \
	-DBUILD_surface=TRUE \
	-DBUILD_outofcore=FALSE \
	-DBUILD_examples=FALSE \
	-DBUILD_simulation=FALSE \
	-DBUILD_registration=TRUE \
	-DBUILD_keypoints=TRUE \
	-DBUILD_tracking=FALSE \
	-DBUILD_recognition=FALSE \
	-DBUILD_stereo=FALSE \
	-DBUILD_visualization=FALSE \
	-DBUILD_tools=FALSE \
    -DCMAKE_BUILD_TYPE=RELEASE \
	"-DCMAKE_INSTALL_PREFIX:PATH=$ALG_DIR/pcl" \
	-DEIGEN_INCLUDE_DIR=$ALG_DIR/eigen3/include/eigen3 \
	-DFLANN_LIBRARY=$ALG_DIR/flann/lib \
	-DFLANN_INCLUDE_DIR=$ALG_DIR/flann/include
RUN make -j$(expr $(nproc) - 2) && make install



# ========== <OpenCV 3.4.3> ==========
WORKDIR /repos
RUN git clone "https://github.com/opencv/opencv.git" opencv
RUN git clone "https://github.com/opencv/opencv_contrib.git" opencv_contrib
WORKDIR /repos/opencv
RUN git checkout 3.4.3
WORKDIR /repos/opencv_contrib
RUN git checkout 3.4.3

# Build opencv
WORKDIR /repos/opencv
RUN mkdir build_dir
WORKDIR build_dir
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE  -D CMAKE_INSTALL_PREFIX=$ALG_DIR/opencv    -DCMAKE_CXX_FLAGS="-Wno-error=address"             \
	-DWITH_GSTREAMER=OFF -DWITH_FFMPEG=OFF -DBUILD_PERF_TESTS=OFF  -D WITH_OPENCL=OFF -D BUILD_WITH_DEBUG_INFO=OFF  -D WITH_1394=OFF                \
	-D BUILD_TESTS=OFF  -D WITH_TBB=OFF  -D WITH_V4L=OFF  -D WITH_OPENGL=OFF -D BUILD_opencv_gpu=OFF    \
	  -D BUILD_opencv_java=OFF -D WITH_CUDA=OFF -DWITH_GTK=ON   -D BUILD_opencv_ml=ON  -D BUILD_opencv_videostab=OFF             \
	   -D BUILD_opencv_ts=OFF    -D BUILD_opencv_photo=ON  -D BUILD_opencv_video=ON -D BUILD_opencv_stitching=OFF -DOPENCV_EXTRA_MODULES_PATH=/repos/opencv_contrib/modules -DENABLE_PRECOMPILED_HEADERS=OFF .. > /repos/opencv/build_dir/opencv_cmake.log
RUN cat /repos/opencv/build_dir/opencv_cmake.log
RUN make -j$(expr $(nproc) - 2) && make install



# ========== <Pangolin> ==========
# Requires: <Eigen3>
RUN apt-get -y install freeglut3 freeglut3-dev libglew-dev libglu1-mesa libglu1-mesa-dev libgl1-mesa-glx libgl1-mesa-dev
WORKDIR /repos
RUN git clone "https://github.com/stevenlovegrove/Pangolin.git" pangolin
WORKDIR pangolin
RUN git checkout c2a6ef524401945b493f14f8b5b8aa76cc7d71a9

# Build pangolin
RUN mkdir build
WORKDIR build
RUN cmake .. -DBUILD_PANGOLIN_VIDEO=OFF  "-DCMAKE_INSTALL_PREFIX:PATH=$ALG_DIR/pangolin"  -DEIGEN_INCLUDE_DIR=$ALG_DIR/eigen3/include/eigen3
RUN make -j$(expr $(nproc) - 2) && make install



# ========== <G2O> ==========
# Requries: <Eigen3>, <Suitesparse>
WORKDIR /repos
RUN git clone "https://github.com/RainerKuemmerle/g2o" g2o
WORKDIR g2o
RUN git checkout 1b118ac2ed2055c4016c3b7cbd710225ed1651af

# Build g2o
RUN mkdir build
WORKDIR build
RUN cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=$ALG_DIR/g2o/" \
	-DCHOLMOD_LIBRARY=$ALG_DIR/suitesparse/lib/libcholmod.a -DCHOLMOD_FOUND=TRUE -DCHOLMOD_INCLUDE_DIR=$ALG_DIR/suitesparse/include/ -DCHOLMOD_LIBRARIES=$ALG_DIR/suitesparse/lib \
	-DCSPARSE_INCLUDE_DIR=$ALG_DIR/suitesparse/include/ -DCSPARSE_LIBRARY=$ALG_DIR/suitesparse/lib/libcxsparse.a \
	-DEIGEN3_INCLUDE_DIR=$ALG_DIR/eigen3/include/eigen3
RUN make -j$(expr $(nproc) - 2) && make install



# ========== <GTSAM> ==========
WORKDIR /repos
RUN wget -O gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip
RUN unzip gtsam.zip

# Build gtsam
WORKDIR gtsam-4.0.0-alpha2
RUN mkdir build
WORKDIR build
RUN cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=$ALG_DIR/gtsam/"
RUN make -j$(expr $(nproc) - 2) install



# ========== <YAML-CPP> ==========
WORKDIR /repos
RUN git clone --branch 0.8.0 https://github.com/jbeder/yaml-cpp.git yaml-cpp

# Build yaml-cpp
WORKDIR yaml-cpp
RUN mkdir build
WORKDIR build
RUN cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=$ALG_DIR/yaml-cpp/"
RUN make -j$(expr $(nproc) - 2) && make install
