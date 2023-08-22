FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

# Declare a variable that represents root directory of the volume
ENV ALG_DIR=/deps/algorithm-name

# Install dependencies with apt-get
RUN apt-get -y update && apt-get -y upgrade
RUN apt-get -y install git
RUN apt-get -y install g++
RUN apt-get -y install cmake
# Boost is commonly used
RUN apt-get -y install libboost-all-dev
# Required by suitesparse
RUN apt-get -y install gfortran libblas-dev liblapack-dev
# Required by ceres
RUN apt-get -y install libgoogle-glog-dev


# /repos/ is directory for source code
RUN mkdir repos


# ====== <Eigen3.2> ======
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


# ====== <Suitesparse v4.3.1> ======
# Required dependencies: gfortran libblas-dev liblapack-dev
WORKDIR /repos
RUN git clone https://github.com/jluttine/suitesparse.git suitesparse
WORKDIR suitesparse
RUN git checkout v4.3.1

# Build suitesparse
RUN make
RUN mkdir $ALG_DIR/suitesparse/include -p
RUN mkdir $ALG_DIR/suitesparse/lib -p
RUN make install INSTALL_LIB=$ALG_DIR/suitesparse/lib INSTALL_INCLUDE=$ALG_DIR/suitesparse/include


# ===== <Ceres> =====
WORKDIR /repos
RUN git clone "https://ceres-solver.googlesource.com/ceres-solver" ceres
WORKDIR ceres
RUN git checkout 1.12.0

# Build ceres
## ceres requires eigen3 and suitesparse,
## so we need to specify the include and lib path of eigen3 and suitesparse.
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


# ====== <SLAMBench> ======
## clone a slambench as we need libslambench-c-wrapper.a
WORKDIR /repos
RUN git clone --branch fake_slambench https://github.com/nikolaradulov/slambench.git slambench

## move libslambench-c-wrapper.a and null.cpp to /deps/algorithm-name/wrapper/
WORKDIR $ALG_DIR
RUN mkdir wrapper
WORKDIR /repos/slambench/docker/wrapper
RUN mv libslambench-c-wrapper.a $ALG_DIR/wrapper
RUN mv null.cpp $ALG_DIR/wrapper


# ====== <Algorithm> ======
WORKDIR /repos
RUN git clone --recursive https://github.com/username/algorithm.git algorithm-name

## "-DCMAKE_INSTALL_PREFIX:PATH=$ALG_DIR/"
### depends on how you write the CMakeLists.txt of the algorithm
### if you have 'install(TARGETS algorithm-library  DESTINATION  lib/)',
### then 'libalgorithm-library.so' will be install at $ALG_DIR/lib/

## "-DLIB_PATH="$ALG_DIR/lib:$ALG_DIR/ceres/lib"
### specify the rpath (could use absolute path), so generated library knows where to find the linked libraries.
### In CMakeList.txt, you would need:
### SET_TARGET_PROPERTIES(${library_name}-library PROPERTIES LINK_FLAGS "-Wl,--disable-new-dtags")
### SET_TARGET_PROPERTIES(${library_name}-library PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE INSTALL_RPATH "${LIB_PATH}")
### NOTE: "-Wl,--disable-new-dtags" makes sure it is rpath not runpath.

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
