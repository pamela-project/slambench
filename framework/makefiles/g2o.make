G2O_DIR=${DEPS_DIR}/g2o
ANDROID_G2O_DIR=${ANDROID_DEPS_DIR}

${SUITE_SPARSE_ROOT}/lib/libcxsparse.a : suitesparse
	echo "please use make suitesparse"

${REPOS_DIR}/g2o :
	mkdir ${REPOS_DIR} -p
	rm ${REPOS_DIR}/g2o -rf
	git clone "https://github.com/RainerKuemmerle/g2o" ${REPOS_DIR}/g2o
	cd ${REPOS_DIR}/g2o  && git checkout 1b118ac2ed2055c4016c3b7cbd710225ed1651af
${DEPS_DIR}/g2o : ${REPOS_DIR}/g2o suitesparse eigen3
	cd ${REPOS_DIR}/g2o && mkdir build -p
	rm ${REPOS_DIR}/g2o/build/* -rf 
	cd ${REPOS_DIR}/g2o/build && cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=$@"  \
	             -DCHOLMOD_LIBRARY=${SUITE_SPARSE_ROOT}/lib/libcholmod.a -DCHOLMOD_FOUND=TRUE -DCHOLMOD_INCLUDE_DIR=${SUITE_SPARSE_ROOT}/include/ -DCHOLMOD_LIBRARIES=${SUITE_SPARSE_ROOT}/lib \
	             -DCSPARSE_INCLUDE_DIR=${SUITE_SPARSE_ROOT}/include/ -DCSPARSE_LIBRARY=${SUITE_SPARSE_ROOT}/lib/libcxsparse.a\
	             -DEIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}
	+cd ${REPOS_DIR}/g2o/build && make
	mkdir -p $@
	cd ${REPOS_DIR}/g2o/build && make install


${ANDROID_DEPS_DIR}/g2o : ${REPOS_DIR}/g2o   ${REPOS_DIR}/android-cmake # What about CHOLMOD and BLAS ?
	cd ${REPOS_DIR}/g2o && mkdir android-build -p
	rm ${REPOS_DIR}/g2o/android-build/* -rf 
	cd ${REPOS_DIR}/g2o/android-build && cmake  -DBUILD_SHARED_LIBS=OFF   -D CMAKE_INSTALL_PREFIX=$(ANDROID_DEPS_DIR)  -DCMAKE_TOOLCHAIN_FILE=${REPOS_DIR}/android-cmake/android.toolchain.cmake -DANDROID_NDK=${ANDROID_NDK} -DCMAKE_BUILD_TYPE=Release -DANDROID_ABI="armeabi-v7a with NEON" -DEIGEN3_INCLUDE_DIR="$(ANDROID_DEPS_DIR)/include/eigen3" -DEIGEN3_VERSION_OK=ON -DG2O_BUILD_EXAMPLES=off -DG2O_BUILD_APPS=off -DCSPARSE_INCLUDE_DIR=${REPOS_DIR}/suitesparse/CXSparse/Include .. 
	+cd ${REPOS_DIR}/g2o/android-build && make
	cd ${REPOS_DIR}/g2o/android-build && make install



g2o : 
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi
android-g2o : ${ANDROID_DEPS_DIR}/g2o

.PHONY: g2o android-g2o
