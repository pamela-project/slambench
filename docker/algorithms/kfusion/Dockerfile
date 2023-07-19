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
RUN git clone --branch fake_slambench https://github.com/nikolaradulov/slambench.git slambench

WORKDIR /deps/kfusion
RUN mkdir wrapper
WORKDIR /repos/slambench/docker/wrapper
RUN mv libslambench-c-wrapper.a /deps/kfusion/wrapper
RUN mv null.cpp /deps/kfusion/wrapper


# ====== <KFusion> ======
WORKDIR /repos
RUN git clone --recursive --branch update-master https://github.com/yuhaozhang7/kfusion.git kfusion


# Generate static library
WORKDIR kfusion
RUN mkdir build
WORKDIR build
RUN cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=/deps/kfusion/" \
	-DWRAPPER_PATH=/deps/kfusion/wrapper/ \
	-DEIGEN3_INCLUDE_DIR=/deps/kfusion/eigen3/include/eigen3/ \
	-DTOON_INCLUDE_PATHS=/deps/kfusion/toon/include/ \
	-DSLAMBENCH_INCLUDE_DIR=/repos/slambench/framework/shared/include/
RUN make && make install

# Script for export lib path
WORKDIR /deps/kfusion
RUN echo "export LD_LIBRARY_PATH=\"/deps/kfusion/lib:$LD_LIBRARY_PATH\"" > libpath.txt

