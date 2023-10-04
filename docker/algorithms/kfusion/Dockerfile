FROM ubuntu:20.04


# Install git and other dependencies
ARG DEBIAN_FRONTEND=noninteractive

ENV ALG_DIR=/deps/kfusion

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
RUN cmake .. -DBUILD_SHARED_LIBS=OFF  -DCMAKE_BUILD_TYPE=Release  "-DCMAKE_INSTALL_PREFIX:PATH=$ALG_DIR/eigen3/"
RUN make -j$(expr $(nproc) - 2) && make install


# ====== <TooN> ======
WORKDIR /repos
RUN git clone "https://github.com/edrosten/TooN.git" TooN
WORKDIR TooN
RUN git checkout 92241416d2a4874fd2334e08a5d417dfea6a1a3f

# Build TooN
RUN ./configure --prefix=$ALG_DIR/toon/
RUN make install


# ====== <SLAMBench> ======
WORKDIR /repos
RUN git clone --branch fake_slambench https://github.com/nikolaradulov/slambench.git slambench

WORKDIR $ALG_DIR
RUN mkdir wrapper
WORKDIR /repos/slambench/docker/wrapper
# RUN mv libslambench-c-wrapper.a $ALG_DIR/wrapper
COPY ./libslambench-c-wrapper.a $ALG_DIR/wrapper/
RUN mv null.cpp $ALG_DIR/wrapper


# ====== <KFusion> ======
WORKDIR /repos
RUN git clone --recursive --branch update-master https://github.com/yuhaozhang7/kfusion-slambench.git kfusion


# Generate static library
WORKDIR kfusion
RUN mkdir build
WORKDIR build
RUN cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=$ALG_DIR/" \
	-DWRAPPER_PATH=$ALG_DIR/wrapper/ \
	-DEIGEN3_INCLUDE_DIR=$ALG_DIR/eigen3/include/eigen3/ \
	-DTOON_INCLUDE_PATHS=$ALG_DIR/toon/include/ \
	-DSLAMBENCH_INCLUDE_DIR=/repos/slambench/framework/shared/include/
RUN make -j$(expr $(nproc) - 2) && make install
