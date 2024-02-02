FROM ubuntu:20.04

# Install git and other dependencies
ARG DEBIAN_FRONTEND=noninteractive
ENV ALG_DIR=/deps/legoloam

RUN apt-get -y update && apt-get -y upgrade
RUN apt-get -y install git
RUN apt-get -y install g++
RUN apt-get -y install curl
RUN apt-get -y install unzip
RUN apt-get -y install cmake
RUN apt-get -y install libboost-all-dev
# Required by pcl
RUN apt-get -y install freeglut3 freeglut3-dev libglew-dev libglu1-mesa libglu1-mesa-dev libgl1-mesa-glx libgl1-mesa-dev

# Make directory for deps and lib
RUN mkdir repos


# ====== <Eigen3.2> ======
WORKDIR /repos
RUN git clone --depth 1 --branch 3.2.10 https://gitlab.com/libeigen/eigen.git eigen3

# Build eigen
WORKDIR eigen3
RUN mkdir build
WORKDIR build
RUN cmake .. -DBUILD_SHARED_LIBS=OFF  -DCMAKE_BUILD_TYPE=Release  "-DCMAKE_INSTALL_PREFIX:PATH=$ALG_DIR/eigen3/"
RUN make -j$(expr $(nproc) - 2) && make install


# ===== <YAML-CPP> =====
WORKDIR /repos
RUN git clone --branch 0.8.0 https://github.com/jbeder/yaml-cpp.git yaml-cpp

# Build yaml-cpp
WORKDIR yaml-cpp
RUN mkdir build
WORKDIR build
RUN cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=$ALG_DIR/yaml-cpp/"
RUN make -j$(expr $(nproc) - 2) && make install


# ===== <GTSAM> =====
WORKDIR /repos
RUN curl -L -o gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip
RUN unzip gtsam.zip

# Build gtsam
WORKDIR gtsam-4.0.0-alpha2
RUN mkdir build
WORKDIR build
RUN cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=$ALG_DIR/gtsam/"
RUN make -j$(expr $(nproc) - 2) install


# ===== <Flann> =====
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


# ===== <PCL 1.8> =====
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


# ====== <OpenCV 3.4.3> ======
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


# ====== <SLAMBench> ======
WORKDIR /repos
RUN git clone --branch fake_slambench https://github.com/nikolaradulov/slambench.git slambench

WORKDIR $ALG_DIR
RUN mkdir wrapper
WORKDIR /repos/slambench/docker/wrapper
# RUN mv libslambench-c-wrapper.a $ALG_DIR/wrapper
COPY ./libslambench-c-wrapper.a $ALG_DIR/wrapper/
RUN mv null.cpp $ALG_DIR/wrapper


# ====== <LeGO-LOAM> ======
WORKDIR /repos
RUN git clone --recursive --branch slambench https://github.com/yuhaozhang7/LeGO-LOAM.git legoloam

WORKDIR /repos/legoloam/LeGO-LOAM-NoROS
RUN mkdir build
WORKDIR build
RUN cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=$ALG_DIR/" \
            -DWRAPPER_PATH=$ALG_DIR/wrapper/ \
            -DSLAMBENCH_INCLUDE_DIR=/repos/slambench/framework/shared/include/ \
            -DEIGEN3_INCLUDE_DIR=$ALG_DIR/eigen3/include/eigen3/ \
            -DFLANN_INCLUDE_DIR=$ALG_DIR/flann/include/ \
            -DPCL_INCLUDE_DIR=$ALG_DIR/pcl/include/pcl-1.8/ \
            -DOpenCV_INCLUDE_DIR=$ALG_DIR/opencv/include/ \
            -DGTSAM_INCLUDE_DIR=$ALG_DIR/gtsam/include/ \
            -DYAML_CPP_INCLUDE_DIR=$ALG_DIR/yaml-cpp/include/ \
            -DFLANN_LIBRARY_DIR=$ALG_DIR/flann/lib/ \
            -DPCL_LIBRARY_DIR=$ALG_DIR/pcl/lib/ \
            -DOpenCV_LIBRARY_DIR=$ALG_DIR/opencv/lib/ \
            -DGTSAM_LIBRARY_DIR=$ALG_DIR/gtsam/lib/ \
            -DYAML_LIBRARY_DIR=$ALG_DIR/yaml-cpp/lib/ \
            -DLIB_PATH="\$ORIGIN:\$ORIGIN/../opencv/lib/:\$ORIGIN/../flann/lib:\$ORIGIN/../pcl/lib/:\$ORIGIN/../gtsam/lib/:\$ORIGIN/../yaml-cpp/lib/"
RUN make -j$(expr $(nproc) - 2) && make install

WORKDIR $ALG_DIR
RUN mkdir configs
WORKDIR configs
RUN mv /repos/legoloam/LeGO-LOAM-NoROS/configs/*.yaml .
