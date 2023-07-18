FROM ubuntu:20.04


# Install git and other dependencies
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get -y update && apt-get -y upgrade
RUN apt-get -y install wget
RUN apt-get -y install git
RUN apt-get -y install g++
RUN apt-get -y install cmake
RUN apt-get -y install freeglut3 freeglut3-dev libglew-dev libglu1-mesa libglu1-mesa-dev libgl1-mesa-glx libgl1-mesa-dev # Dependency of pangolin

# Make directory for deps and lib
RUN mkdir repos


# ====== <TooN> ======
WORKDIR /repos
RUN git clone "https://github.com/edrosten/TooN.git" TooN
WORKDIR TooN
RUN git checkout 92241416d2a4874fd2334e08a5d417dfea6a1a3f

# Build TooN
RUN ./configure --prefix=/deps/orbslam2/toon/
RUN make install


# ====== <Eigen3.2> ======
WORKDIR /repos
RUN git clone --depth 1 --branch 3.2.10 https://gitlab.com/libeigen/eigen.git eigen3

# Build eigen
WORKDIR eigen3
RUN mkdir build
WORKDIR build
RUN cmake .. -DBUILD_SHARED_LIBS=OFF  -DCMAKE_BUILD_TYPE=Release  "-DCMAKE_INSTALL_PREFIX:PATH=/deps/orbslam2/eigen3/"
RUN make -j2 && make install

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
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE  -D CMAKE_INSTALL_PREFIX=/deps/orbslam2/opencv    -DCMAKE_CXX_FLAGS="-Wno-error=address"             \
	-DWITH_GSTREAMER=OFF -DWITH_FFMPEG=OFF -DBUILD_PERF_TESTS=OFF  -D WITH_OPENCL=OFF -D BUILD_WITH_DEBUG_INFO=OFF  -D WITH_1394=OFF                \
	-D BUILD_TESTS=OFF  -D WITH_TBB=OFF  -D WITH_V4L=OFF  -D WITH_OPENGL=OFF -D BUILD_opencv_gpu=OFF    \
	  -D BUILD_opencv_java=OFF -D WITH_CUDA=OFF -DWITH_GTK=ON   -D BUILD_opencv_ml=ON  -D BUILD_opencv_videostab=OFF             \
	   -D BUILD_opencv_ts=OFF    -D BUILD_opencv_photo=ON  -D BUILD_opencv_video=ON -D BUILD_opencv_stitching=OFF -DOPENCV_EXTRA_MODULES_PATH=/repos/opencv_contrib/modules -DENABLE_PRECOMPILED_HEADERS=OFF .. > /repos/opencv/build_dir/opencv_cmake.log
RUN cat /repos/opencv/build_dir/opencv_cmake.log
RUN make -j2 && make install


# ====== <Pangolin> ======
# Required dependencies: opengl, libglew-dev, eigen3
WORKDIR /repos
RUN git clone "https://github.com/stevenlovegrove/Pangolin.git" pangolin
WORKDIR pangolin
RUN git checkout c2a6ef524401945b493f14f8b5b8aa76cc7d71a9

# Build pangolin
RUN mkdir build
WORKDIR build
RUN cmake .. -DBUILD_PANGOLIN_VIDEO=OFF  "-DCMAKE_INSTALL_PREFIX:PATH=/deps/orbslam2/pangolin"  -DEIGEN_INCLUDE_DIR=/deps/orbslam2/eigen3/include/eigen3
RUN make -j2 && make install


# ====== <SLAMBench> ======
WORKDIR /repos
RUN git clone https://github.com/nikolaradulov/slambench.git slambench


# ====== <ORB-SLAM2> ======
WORKDIR /repos
RUN git clone --recursive --branch update-master https://github.com/yuhaozhang7/ORB_SLAM2.git orbslam2

# Generate static library
WORKDIR orbslam2
RUN mkdir build
WORKDIR build
RUN cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=/deps/orbslam2/" \
	-DCMAKE_PREFIX_PATH="/deps/orbslam2/opencv" \
	-DTOON_INCLUDE_DIR=/deps/orbslam2/toon/include/ \
	-DEIGEN3_INCLUDE_DIR=/deps/orbslam2/eigen3/include/eigen3/ \
	-DPangolin_INCLUDE_DIRS=/deps/orbslam2/pangolin/include/ \
	-DSLAMBENCH_INCLUDE_DIRS=/repos/slambench/framework/shared/include/ \
	-DPangolin_LIBRARIES=/deps/orbslam2/pangolin/lib/libpangolin.so
RUN make && make install
# Move the vocabulary file
WORKDIR /deps/orbslam2
RUN mkdir Vocabulary
WORKDIR /repos/orbslam2/Vocabulary
RUN mv ORBvoc.bin /deps/orbslam2/Vocabulary


# CMakeLists for generating dynamic library
WORKDIR /deps/orbslam2

RUN echo "cmake_minimum_required(VERSION 2.8)" > CMakeLists.txt
RUN echo "project(liborbslam2)" >> CMakeLists.txt
RUN echo "" >> CMakeLists.txt

RUN echo "SET(CMAKE_C_FLAGS \"\${CMAKE_C_FLAGS} -g -O3                  -Wall -Wno-unknown-pragmas -Wextra -Wno-error=deprecated-declarations -Wno-error=array-bounds\")" >> CMakeLists.txt
RUN echo "SET(CMAKE_CXX_FLAGS \"\${CMAKE_CXX_FLAGS} -g -O3  --std=c++11 -Wall -Wno-unknown-pragmas -Wextra -Wno-error=deprecated-declarations -Wno-error=array-bounds\")" >> CMakeLists.txt
RUN echo "" >> CMakeLists.txt

RUN echo "SET(LIBNAME orbslam2-original-library)" >> CMakeLists.txt
RUN echo "SET(SLAMBENCH_C_WRAPPER	-Wl,--whole-archive -L/slambench/build/lib/ slambench-c-wrapper	-Wl,--no-whole-archive)" >> CMakeLists.txt
RUN echo "SET(NULL_FILE_PATH	\"/slambench/build/null.cpp\")" >> CMakeLists.txt
RUN echo "" >> CMakeLists.txt

RUN echo "find_package(OpenCV 3.0 QUIET)" >> CMakeLists.txt
RUN echo "set(ORB_SLAM2_LIB	/deps/orbslam2/lib/liborbslam2.so)" >> CMakeLists.txt
RUN echo "set(Pangolin_LIBRARIES	/deps/orbslam2/pangolin/lib/libpangolin.so)" >> CMakeLists.txt
RUN echo "set(DBOW2_LIB	/deps/orbslam2/DBoW2/libDBoW2.so)" >> CMakeLists.txt
RUN echo "set(G20_LIB		/deps/orbslam2/g2o/libg2o.so)" >> CMakeLists.txt
RUN echo "set(common_libraries     \${ORB_SLAM2_LIB} \${OpenCV_LIBS} \${Pangolin_LIBRARIES} \${DBOW2_LIB} \${G20_LIB})" >> CMakeLists.txt
RUN echo "" >> CMakeLists.txt

RUN echo "add_library(\${LIBNAME} SHARED \${NULL_FILE_PATH})" >> CMakeLists.txt
RUN echo "target_link_libraries(\${LIBNAME} PRIVATE \${SLAMBENCH_C_WRAPPER} /deps/orbslam2/lib/liborbslam2-original.so \${common_libraries})" >> CMakeLists.txt
RUN echo "SET_TARGET_PROPERTIES(\${LIBNAME} PROPERTIES LINK_FLAGS \"\${ARGN}\")" >> CMakeLists.txt
RUN echo "install(TARGETS \${LIBNAME}  DESTINATION  lib/)" >> CMakeLists.txt
RUN echo "" >> CMakeLists.txt


