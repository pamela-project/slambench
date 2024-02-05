FROM ubuntu:20.04


# Install git and other dependencies
ARG DEBIAN_FRONTEND=noninteractive
ENV ORB3_DIR=/deps/orbslam3

RUN apt-get -y update && apt-get -y upgrade
RUN apt-get -y install git
RUN apt-get -y install g++
RUN apt-get -y install cmake
RUN apt-get -y install freeglut3 freeglut3-dev libglew-dev libglu1-mesa libglu1-mesa-dev libgl1-mesa-glx libgl1-mesa-dev # Dependency of pangolin
RUN apt-get -y install libboost-all-dev libssl-dev # for -lboost_serialization -lcrypto

# Make directory for deps and lib
RUN mkdir repos


# ====== <TooN> ======
WORKDIR /repos
RUN git clone "https://github.com/edrosten/TooN.git" TooN
WORKDIR TooN
RUN git checkout 92241416d2a4874fd2334e08a5d417dfea6a1a3f

# Build TooN
RUN ./configure --prefix=$ORB3_DIR/toon/
RUN make install


# ====== <Eigen3.2> ======
WORKDIR /repos
RUN git clone --depth 1 --branch 3.2.10 https://gitlab.com/libeigen/eigen.git eigen3

# Build eigen
WORKDIR eigen3
RUN mkdir build
WORKDIR build
RUN cmake .. -DBUILD_SHARED_LIBS=OFF  -DCMAKE_BUILD_TYPE=Release  "-DCMAKE_INSTALL_PREFIX:PATH=$ORB3_DIR/eigen3/"
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
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE  -D CMAKE_INSTALL_PREFIX=$ORB3_DIR/opencv    -DCMAKE_CXX_FLAGS="-Wno-error=address"             \
	-DWITH_GSTREAMER=OFF -DWITH_FFMPEG=OFF -DBUILD_PERF_TESTS=OFF  -D WITH_OPENCL=OFF -D BUILD_WITH_DEBUG_INFO=OFF  -D WITH_1394=OFF                \
	-D BUILD_TESTS=OFF  -D WITH_TBB=OFF  -D WITH_V4L=OFF  -D WITH_OPENGL=OFF -D BUILD_opencv_gpu=OFF    \
	  -D BUILD_opencv_java=OFF -D WITH_CUDA=OFF -DWITH_GTK=ON   -D BUILD_opencv_ml=ON  -D BUILD_opencv_videostab=OFF             \
	   -D BUILD_opencv_ts=OFF    -D BUILD_opencv_photo=ON  -D BUILD_opencv_video=ON -D BUILD_opencv_stitching=OFF -DOPENCV_EXTRA_MODULES_PATH=/repos/opencv_contrib/modules -DENABLE_PRECOMPILED_HEADERS=OFF .. > /repos/opencv/build_dir/opencv_cmake.log
RUN cat /repos/opencv/build_dir/opencv_cmake.log
RUN make -j$(expr $(nproc) - 2) && make install


# ====== <Pangolin> ======
# Required dependencies: opengl, libglew-dev, eigen3
WORKDIR /repos
RUN git clone "https://github.com/stevenlovegrove/Pangolin.git" pangolin
WORKDIR pangolin
RUN git checkout c2a6ef524401945b493f14f8b5b8aa76cc7d71a9

# Build pangolin
RUN mkdir build
WORKDIR build
RUN cmake .. -DBUILD_PANGOLIN_VIDEO=OFF  "-DCMAKE_INSTALL_PREFIX:PATH=$ORB3_DIR/pangolin"  -DEIGEN_INCLUDE_DIR=$ORB3_DIR/eigen3/include/eigen3
RUN make -j$(expr $(nproc) - 2) && make install


# ====== <SLAMBench> ======
WORKDIR /repos
RUN git clone --branch fake_slambench https://github.com/nikolaradulov/slambench.git slambench

WORKDIR $ORB3_DIR
RUN mkdir wrapper && mkdir Vocabulary
WORKDIR /repos/slambench/docker/wrapper
# RUN mv libslambench-c-wrapper.a $ORB3_DIR/wrapper
COPY ./libslambench-c-wrapper.a $ORB3_DIR/wrapper/
RUN mv null.cpp $ORB3_DIR/wrapper


# ====== <ORB-SLAM3> ======
WORKDIR /repos
RUN git clone --recursive --branch master https://github.com/yuhaozhang7/orbslam3-slambench.git orbslam3

WORKDIR orbslam3
RUN mkdir build
WORKDIR build
RUN cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=$ORB3_DIR/" \
	-DCMAKE_PREFIX_PATH="$ORB3_DIR/opencv" \
	-DWRAPPER_PATH=$ORB3_DIR/wrapper/ \
	-DTOON_INCLUDE_DIR=$ORB3_DIR/toon/include/ \
	-DEIGEN3_INCLUDE_DIR=$ORB3_DIR/eigen3/include/eigen3/ \
	-DPangolin_INCLUDE_DIRS=$ORB3_DIR/pangolin/include/ \
	-DSLAMBENCH_INCLUDE_DIRS=/repos/slambench/framework/shared/include/ \
	-DPangolin_LIBRARIES=$ORB3_DIR/pangolin/lib/libpangolin.so \
	-DLIB_PATH="\$ORIGIN:\$ORIGIN/../pangolin/lib:\$ORIGIN/../opencv/lib:\$ORIGIN/../DBoW2:\$ORIGIN/../g2o"
RUN make -j$(expr $(nproc) - 2) && make install

# Move the vocabulary file
WORKDIR /repos/orbslam3/Vocabulary
RUN tar -xf ORBvoc.txt.tar.gz
RUN mv ORBvoc.txt $ORB3_DIR/Vocabulary
