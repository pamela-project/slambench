FROM ubuntu:20.04


# Install git and other dependencies
ARG DEBIAN_FRONTEND=noninteractive
ENV OPENVINS_DIR=/deps/openvins

RUN apt-get -y update && apt-get -y upgrade
RUN apt-get -y install wget
RUN apt-get -y install git
RUN apt-get -y install g++
RUN apt-get -y install cmake
RUN apt-get -y install libboost-all-dev

# Make directory for deps and lib
RUN mkdir repos


# ====== <Eigen3.2> ======
WORKDIR /repos
RUN git clone --depth 1 --branch 3.2.10 https://gitlab.com/libeigen/eigen.git eigen3

# Build eigen
WORKDIR eigen3
RUN mkdir build
WORKDIR build
RUN cmake .. -DBUILD_SHARED_LIBS=OFF  -DCMAKE_BUILD_TYPE=Release  "-DCMAKE_INSTALL_PREFIX:PATH=$OPENVINS_DIR/eigen3/"
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
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE  -D CMAKE_INSTALL_PREFIX=$OPENVINS_DIR/opencv    -DCMAKE_CXX_FLAGS="-Wno-error=address"             \
	-DWITH_GSTREAMER=OFF -DWITH_FFMPEG=OFF -DBUILD_PERF_TESTS=OFF  -D WITH_OPENCL=OFF -D BUILD_WITH_DEBUG_INFO=OFF  -D WITH_1394=OFF                \
	-D BUILD_TESTS=OFF  -D WITH_TBB=OFF  -D WITH_V4L=OFF  -D WITH_OPENGL=OFF -D BUILD_opencv_gpu=OFF    \
	  -D BUILD_opencv_java=OFF -D WITH_CUDA=OFF -DWITH_GTK=ON   -D BUILD_opencv_ml=ON  -D BUILD_opencv_videostab=OFF             \
	   -D BUILD_opencv_ts=OFF    -D BUILD_opencv_photo=ON  -D BUILD_opencv_video=ON -D BUILD_opencv_stitching=OFF -DOPENCV_EXTRA_MODULES_PATH=/repos/opencv_contrib/modules -DENABLE_PRECOMPILED_HEADERS=OFF .. > /repos/opencv/build_dir/opencv_cmake.log
RUN cat /repos/opencv/build_dir/opencv_cmake.log
RUN make -j$(expr $(nproc) - 2) && make install


# ====== <SLAMBench> ======
WORKDIR /repos
RUN git clone --branch fake_slambench https://github.com/nikolaradulov/slambench.git slambench

WORKDIR $OPENVINS_DIR
RUN mkdir wrapper
WORKDIR /repos/slambench/docker/wrapper
# RUN mv libslambench-c-wrapper.a $OPENVINS_DIR/wrapper
COPY ./libslambench-c-wrapper.a $OPENVINS_DIR/wrapper/
RUN mv null.cpp $OPENVINS_DIR/wrapper


# ====== <Open-VINS> ======
WORKDIR /repos
RUN git clone --recursive --branch rpng-master https://github.com/yuhaozhang7/openvins-slambench.git openvins

# Generate static library
WORKDIR openvins
RUN mkdir build
WORKDIR build
RUN cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=$OPENVINS_DIR/" \
	-DCMAKE_PREFIX_PATH="$OPENVINS_DIR/opencv" \
	-DWRAPPER_PATH=$OPENVINS_DIR/wrapper/ \
	-DEIGEN3_INCLUDE_DIR=$OPENVINS_DIR/eigen3/include/eigen3/ \
	-DSLAMBENCH_INCLUDE_DIRS=/repos/slambench/framework/shared/include/ \
	-DLIB_PATH="\$ORIGIN:\$ORIGIN/../opencv/lib"
RUN make -j$(expr $(nproc) - 2) && make install
