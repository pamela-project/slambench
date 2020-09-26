EIGEN3_INCLUDE_DIR=${DEPS_DIR}/eigen3/include/eigen3
ANDROID_EIGEN3_INCLUDE_DIR=${ANDROID_DEPS_DIR}/eigen3/include/eigen3

${REPOS_DIR}/eigen3 :
	mkdir ${REPOS_DIR}/ -p
	rm ${REPOS_DIR}/eigen3 -rf
	cd ${REPOS_DIR}/ && git clone --depth 1 --branch 3.2.10 https://gitlab.com/libeigen/eigen.git eigen3


${DEPS_DIR}/eigen3  : ${REPOS_DIR}/eigen3 
	cd ${REPOS_DIR}/eigen3 && mkdir build_dir -p
	cd ${REPOS_DIR}/eigen3 && rm build_dir/* -rf
	cd ${REPOS_DIR}/eigen3/build_dir && cmake .. -DBUILD_SHARED_LIBS=OFF  -DCMAKE_BUILD_TYPE=Release  "-DCMAKE_INSTALL_PREFIX:PATH=$@" 
	+cd ${REPOS_DIR}/eigen3/build_dir && make
	mkdir ${DEPS_DIR}/eigen3 -p
	cd ${REPOS_DIR}/eigen3/build_dir && make install



${ANDROID_DEPS_DIR}/eigen3  : ${REPOS_DIR}/eigen3
	mkdir $@/include/eigen3 -p
	cp -rf ${REPOS_DIR}/eigen3/Eigen $@/include/eigen3

eigen : eigen3

eigen3 :
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi
android-eigen3 : ${ANDROID_DEPS_DIR}/eigen3

.PHONY: eigen eigen3 android-eigen3
