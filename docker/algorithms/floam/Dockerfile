FROM ubuntu:20.04


# Install git and other dependencies
ARG DEBIAN_FRONTEND=noninteractive
ENV LOAM_DIR=/deps/floam

RUN apt-get -y update && apt-get -y upgrade
RUN apt-get -y install git
RUN apt-get -y install g++
RUN apt-get -y install cmake
RUN apt-get -y install libboost-all-dev libyaml-cpp-dev
# Required by suitesparse
RUN apt-get -y install gfortran libblas-dev liblapack-dev
# Required by ceres
RUN apt-get -y install libgoogle-glog-dev
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
RUN cmake .. -DBUILD_SHARED_LIBS=OFF  -DCMAKE_BUILD_TYPE=Release  "-DCMAKE_INSTALL_PREFIX:PATH=$LOAM_DIR/eigen3/"
RUN make -j$(expr $(nproc) - 2) && make install


# ===== <YAML-CPP> =====
WORKDIR /repos
RUN git clone --branch 0.8.0 https://github.com/jbeder/yaml-cpp.git yaml-cpp

# Build yaml-cpp
WORKDIR yaml-cpp
RUN mkdir build
WORKDIR build
RUN cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=$LOAM_DIR/yaml-cpp/"
RUN make -j$(expr $(nproc) - 2) && make install


# ====== <Suitesparse v4.3.1> ======
# Required dependencies: gfortran libblas-dev liblapack-dev
WORKDIR /repos
RUN git clone https://github.com/jluttine/suitesparse.git suitesparse
WORKDIR suitesparse
RUN git checkout v4.3.1

# Build suitesparse
RUN make
RUN mkdir $LOAM_DIR/suitesparse/include -p
RUN mkdir $LOAM_DIR/suitesparse/lib -p
RUN make install INSTALL_LIB=$LOAM_DIR/suitesparse/lib INSTALL_INCLUDE=$LOAM_DIR/suitesparse/include


# ===== <Ceres> =====
WORKDIR /repos
RUN git clone "https://ceres-solver.googlesource.com/ceres-solver" ceres
WORKDIR ceres
RUN git checkout 1.12.0

# build ceres
RUN mkdir build
WORKDIR build
RUN cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=$LOAM_DIR/ceres" \
             "-DBUILD_EXAMPLES:BOOL=OFF" \
             "-DBUILD_TESTING:BOOL=OFF" \
             "-DBUILD_DOCUMENTATION=OFF" \
             "-DBUILD_SHARED_LIBS=ON" \
             "-DCERES_USING_SHARED_LIBRARY=OFF" \
             -DCMAKE_CXX_FLAGS="-w -O3 -std=c++11" \
             -DEIGEN_INCLUDE_DIR=$LOAM_DIR/eigen3/include/eigen3 \
             -DSUITESPARSE_INCLUDE_DIR_HINTS=$LOAM_DIR/suitesparse/include \
             -DSUITESPARSE_LIBRARY_DIR_HINTS=$LOAM_DIR/suitesparse/lib \
             -DCXSPARSE_INCLUDE_DIR=$LOAM_DIR/suitesparse/include \
             -DCXSPARSE_LIBRARY=$LOAM_DIR/suitesparse/lib/libcxsparse.a
RUN make -j$(expr $(nproc) - 2) && make install


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
RUN cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=$LOAM_DIR/flann/" \
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
	"-DCMAKE_INSTALL_PREFIX:PATH=$LOAM_DIR/pcl" \
	-DEIGEN_INCLUDE_DIR=$LOAM_DIR/eigen3/include/eigen3 \
	-DFLANN_LIBRARY=$LOAM_DIR/flann/lib \
	-DFLANN_INCLUDE_DIR=$LOAM_DIR/flann/include
RUN make -j$(expr $(nproc) - 2) && make install


# ====== <SLAMBench> ======
WORKDIR /repos
RUN git clone --branch fake_slambench https://github.com/nikolaradulov/slambench.git slambench

WORKDIR $LOAM_DIR
RUN mkdir wrapper
WORKDIR /repos/slambench/docker/wrapper
# RUN mv libslambench-c-wrapper.a $LOAM_DIR/wrapper
COPY ./libslambench-c-wrapper.a $LOAM_DIR/wrapper/
RUN mv null.cpp $LOAM_DIR/wrapper


# ====== <F-LOAM> ======
WORKDIR /repos
RUN git clone --recursive --branch slambench https://github.com/yuhaozhang7/floam.git floam

WORKDIR floam
RUN mkdir build
WORKDIR build
RUN cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=$LOAM_DIR/" \
 	-DWRAPPER_PATH=$LOAM_DIR/wrapper/ \
 	-DEIGEN3_INCLUDE_DIR=$LOAM_DIR/eigen3/include/eigen3/ \
	-DFLANN_INCLUDE_DIR=$LOAM_DIR/flann/include/ \
    -DPCL_INCLUDE_DIR=$LOAM_DIR/pcl/include/pcl-1.8/ \
    -DCERES_INCLUDE_DIR=$LOAM_DIR/ceres/include/ \
    -DYAML_CPP_INCLUDE_DIR=$LOAM_DIR/yaml-cpp/include/ \
 	-DSLAMBENCH_INCLUDE_DIR=/repos/slambench/framework/shared/include/ \
	-DFLANN_LIBRARY_DIR=$LOAM_DIR/flann/lib/ \
    -DPCL_LIBRARY_DIR=$LOAM_DIR/pcl/lib/ \
    -DCERES_LIBRARY_DIR=$LOAM_DIR/ceres/lib/ \
    -DYAML_CPP_LIBRARY_DIR=$LOAM_DIR/yaml-cpp/lib/ \
 	-DLIB_PATH="\$ORIGIN:\$ORIGIN/../flann/lib:\$ORIGIN/../pcl/lib:\$ORIGIN/../ceres/lib:\$ORIGIN/../yaml-cpp/lib"
RUN make -j$(expr $(nproc) - 2) && make install

WORKDIR $LOAM_DIR
RUN mkdir configs
WORKDIR configs
RUN mv /repos/floam/configs/*.yaml .
