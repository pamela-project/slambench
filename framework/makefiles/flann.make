FLANN_INCLUDE_DIR=${DEPS_DIR}/flann/include
FLANN_LIBRARY=${DEPS_DIR}/flann/lib/libflann.so
#include eigen3 # not required

${REPOS_DIR}/flann :
	mkdir -p ${REPOS_DIR}
	rm -rf ${REPOS_DIR}/flann	
	cd ${REPOS_DIR} ; git clone git://github.com/mariusmuja/flann.git flann
	cd ${REPOS_DIR}/flann && git checkout 06a49513138009d19a1f4e0ace67fbff13270c69
	touch ${REPOS_DIR}/flann/src/cpp/empty.cpp
	sed -e "/add_library(flann_cpp SHARED/ s/\"\"/empty.cpp/" -e "/add_library(flann SHARED/ s/\"\"/empty.cpp/" -i ${REPOS_DIR}/flann/src/cpp/CMakeLists.txt

${DEPS_DIR}/flann : ${REPOS_DIR}/flann
	mkdir -p ${REPOS_DIR}/flann/build 
	rm -rf ${REPOS_DIR}/flann/buid/* 
	cd ${REPOS_DIR}/flann/build && cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=$@" -DBUILD_MATLAB_BINDINGS=FALSE -DBUILD_PYTHON_BINDINGS=FALSE -DBUILD_EXAMPLES=FALSE -DBUILD_TESTS=FALSE -DBUILD_DOC=FALSE
	+cd ${REPOS_DIR}/flann/build && make 
	mkdir -p $@
	cd ${REPOS_DIR}/flann/build && make install

flann :
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi
.PHONY: flann
