EIGEN3_INCLUDE_DIR=${DEPS_DIR}/eigen3/include/eigen3
ANDROID_EIGEN3_INCLUDE_DIR=${ANDROID_DEPS_DIR}/eigen3/include/eigen3

${REPOS_DIR}/eigen3 :
	mkdir -p ${REPOS_DIR}/
	rm -rf ${REPOS_DIR}/eigen3
	cd ${REPOS_DIR}/ && hg clone --insecure "http://bitbucket.org/eigen/eigen" eigen3
	cd ${REPOS_DIR}/eigen3 && hg update 3.2


${DEPS_DIR}/eigen3  : ${REPOS_DIR}/eigen3 
	cd ${REPOS_DIR}/eigen3 && mkdir -p build_dir
	cd ${REPOS_DIR}/eigen3 && rm -rf build_dir/* 
	cd ${REPOS_DIR}/eigen3/build_dir && cmake .. -DBUILD_SHARED_LIBS=OFF  -DCMAKE_BUILD_TYPE=Release  "-DCMAKE_INSTALL_PREFIX:PATH=$@" 
	+cd ${REPOS_DIR}/eigen3/build_dir && make
	mkdir -p ${DEPS_DIR}/eigen3
	cd ${REPOS_DIR}/eigen3/build_dir && make install



${ANDROID_DEPS_DIR}/eigen3  : ${REPOS_DIR}/eigen3
	mkdir -p $@/include/eigen3
	cp -rf ${REPOS_DIR}/eigen3/Eigen $@/include/eigen3

eigen : eigen3

eigen3 :
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi
android-eigen3 : ${ANDROID_DEPS_DIR}/eigen3

.PHONY: eigen eigen3 android-eigen3
