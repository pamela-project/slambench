Sophus_INCLUDE_DIR=${DEPS_DIR}/Sophus/include
Sophus_INCLUDE_DIRS=${DEPS_DIR}/Sophus/include
Sophus_DIR=${DEPS_DIR}/Sophus/share/sophus/cmake

${REPOS_DIR}/Sophus :
	mkdir ${REPOS_DIR} -p
	rm ${REPOS_DIR}/Sophus -rf
	git clone "https://github.com/strasdat/Sophus.git" ${REPOS_DIR}/Sophus
	cd ${REPOS_DIR}/Sophus && git checkout b474f05f839c0f63c281aa4e7ece03145729a2cd


${DEPS_DIR}/Sophus : ${REPOS_DIR}/Sophus
	cd ${REPOS_DIR}/Sophus && mkdir build -p && rm build/* -rf
	cd ${REPOS_DIR}/Sophus/build && cmake .. "-DCMAKE_INSTALL_PREFIX=$@" -DCMAKE_BUILD_TYPE=Release
	cd ${REPOS_DIR}/Sophus/build && make -j2
	mkdir -p $@
	cd ${REPOS_DIR}/Sophus/build && make install

Sophus :
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi
.PHONY: sophus
