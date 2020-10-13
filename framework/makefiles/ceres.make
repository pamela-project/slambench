CERES_DIR=${DEPS_DIR}/ceres/

#include eigen3 # not required

${REPOS_DIR}/ceres :
	mkdir -p ${REPOS_DIR}
	rm -rf ${REPOS_DIR}/ceres	
	cd ${REPOS_DIR} ; git clone "https://ceres-solver.googlesource.com/ceres-solver" ceres
	cd $@ && git checkout 1.12.0

${DEPS_DIR}/ceres : ${REPOS_DIR}/ceres eigen3 suitesparse
	mkdir ${REPOS_DIR}/ceres/build -p
	rm ${REPOS_DIR}/ceres/build/* -rf
	cd ${REPOS_DIR}/ceres/build && cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=$@" "-DBUILD_EXAMPLES:BOOL=OFF" "-DBUILD_TESTING:BOOL=OFF" \
	                                         -DCMAKE_CXX_FLAGS="-w -O3 -std=c++11" -DEIGEN_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR} \
	                                         -DSUITESPARSE_INCLUDE_DIR_HINTS=${SUITE_SPARSE_ROOT}/include\
	                                         -DSUITESPARSE_LIBRARY_DIR_HINTS=${SUITE_SPARSE_ROOT}/lib\
	                                         -DBUILD_DOCUMENTATION=OFF -DBUILD_SHARED_LIBS=ON\
	                                         -DCXSPARSE_INCLUDE_DIR=${SUITE_SPARSE_ROOT}/include -DCXSPARSE_LIBRARY=${SUITE_SPARSE_ROOT}/lib/libcxsparse.a\
	                                         -DCERES_USING_SHARED_LIBRARY=OFF
#	                                         -DGFLAGS=OFF
#	                                         -DSUITESPARSE=OFF
#	                                         -DCXSPARSE=OFF
#	                                         -DLAPACK=OFF
#	                                         
#	                                         -DOPENMP=OFF
	cd ${REPOS_DIR}/ceres/build && make 	
	mkdir -p $@
	cd ${REPOS_DIR}/ceres/build && make install

ceres :
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi
.PHONY: ceres
