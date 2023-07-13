FROM ubuntu:20.04


# Install git and other dependencies
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get -y update && apt-get -y upgrade
RUN apt-get -y install wget
RUN apt-get -y install git
RUN apt-get -y install g++
RUN apt-get -y install cmake

# Make directory for deps and lib
RUN mkdir repos


# ====== <Eigen3.2> ======
WORKDIR /repos
RUN git clone --depth 1 --branch 3.2.10 https://gitlab.com/libeigen/eigen.git eigen3

# Build eigen
WORKDIR eigen3
RUN mkdir build
WORKDIR build
RUN cmake .. -DBUILD_SHARED_LIBS=OFF  -DCMAKE_BUILD_TYPE=Release  "-DCMAKE_INSTALL_PREFIX:PATH=/deps/kfusion/eigen3/"
RUN make && make install


# ====== <TooN> ======
WORKDIR /repos
RUN git clone "https://github.com/edrosten/TooN.git" TooN
WORKDIR TooN
RUN git checkout 92241416d2a4874fd2334e08a5d417dfea6a1a3f

# Build TooN
RUN ./configure --prefix=/deps/kfusion/toon/
RUN make install


# ====== <SLAMBench> ======
WORKDIR /repos
RUN git clone https://github.com/nikolaradulov/slambench.git slambench


# ====== <KFusion> ======
WORKDIR /repos
RUN git clone --branch update-master https://github.com/yuhaozhang7/kfusion.git kfusion


# Generate static library
WORKDIR kfusion
RUN mkdir build
WORKDIR build
RUN cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=/deps/kfusion/" \
	-DEIGEN3_INCLUDE_DIR=/deps/kfusion/eigen3/include/eigen3/ \
	-DTOON_INCLUDE_PATHS=/deps/kfusion/toon/include/ \
	-DSLAMBENCH_INCLUDE_DIR=/repos/slambench/framework/shared/include/
RUN make && make install

# CMakeLists for generating dynamic library
WORKDIR /deps/kfusion

RUN echo "cmake_minimum_required(VERSION 2.8)" > CMakeLists.txt
RUN echo "project(libkfusion)" >> CMakeLists.txt
RUN echo "" >> CMakeLists.txt

RUN echo "SET(CMAKE_C_FLAGS \"\${CMAKE_C_FLAGS} -g -O3                  -Wall -Wno-unknown-pragmas -Wextra -Wno-error=deprecated-declarations -Wno-error=array-bounds\")" >> CMakeLists.txt
RUN echo "SET(CMAKE_CXX_FLAGS \"\${CMAKE_CXX_FLAGS} -g -O3  --std=c++11 -Wall -Wno-unknown-pragmas -Wextra -Wno-error=deprecated-declarations -Wno-error=array-bounds\")" >> CMakeLists.txt
RUN echo "" >> CMakeLists.txt

RUN echo "SET(LIBNAME kfusion-cpp-library)" >> CMakeLists.txt
RUN echo "SET(PATH_TO_STATIC /deps/kfusion/lib/libkfusion-cpp.a)" >> CMakeLists.txt
RUN echo "SET(SLAMBENCH_C_WRAPPER	-Wl,--whole-archive -L/slambench/build/lib/ slambench-c-wrapper	-Wl,--no-whole-archive)" >> CMakeLists.txt
RUN echo "SET(NULL_FILE_PATH	\"/slambench/build/null.cpp\")" >> CMakeLists.txt
RUN echo "" >> CMakeLists.txt

RUN echo "add_library(\${LIBNAME} SHARED \${NULL_FILE_PATH})" >> CMakeLists.txt
RUN echo "target_link_libraries(\${LIBNAME} PRIVATE \${SLAMBENCH_C_WRAPPER} \${PATH_TO_STATIC})" >> CMakeLists.txt
RUN echo "SET_TARGET_PROPERTIES(\${LIBNAME} PROPERTIES LINK_FLAGS \"\${ARGN}\")" >> CMakeLists.txt
RUN echo "install(TARGETS \${LIBNAME}  DESTINATION  lib/)" >> CMakeLists.txt
RUN echo "" >> CMakeLists.txt

