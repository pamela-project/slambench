

GCC7_COMPILER=${DEPS_DIR}/gcc7/bin/c++
GCC5_COMPILER=${DEPS_DIR}/gcc5/bin/c++

GCC_REPOS=gcc
GCC_BUILD_DIR=${DEPS_DIR}/build/gcc
GCC_7_SRC_DIR=gcc7
GCC7_TGZ=gcc-7.2.0.tar.gz
GCC7_TGZ_URL=https://github.com/gcc-mirror/gcc/archive/releases/${GCC7_TGZ}


${REPOS_DIR}/${GCC_REPOS} :
	mkdir -p ${REPOS_DIR}
	rm -rf ${REPOS_DIR}/${GCC_REPOS}
	cd ${REPOS_DIR} && git clone git://gcc.gnu.org/git/gcc.git ${GCC_REPOS}

${REPOS_DIR}/gcc7/ : 
	mkdir -p ${REPOS_DIR}
	rm -rf $@
	cd ${REPOS_DIR} && wget ${GCC7_TGZ_URL} && tar xzf ${GCC7_TGZ} && mv gcc-releases-gcc-7.2.0 gcc7/

${DEPS_DIR}/gcc5 : ${REPOS_DIR}/${GCC_REPOS}
	cd $^ && git checkout releases/gcc-5.3.0
	cd $^ && git cherry-pick ec1cc0263f156f70693a62cf17b254a0029f4852 || true
	cd $^ && ./contrib/download_prerequisites
	mkdir -p ${GCC_BUILD_DIR}/objdir
	rm -rf ${GCC_BUILD_DIR}/objdir/*
	cd  ${GCC_BUILD_DIR}/objdir && ${REPOS_DIR}/${GCC_REPOS}/configure --prefix=$@ --enable-languages=c,c++ --disable-multilib
	+cd ${GCC_BUILD_DIR}/objdir && make
	mkdir -p $@
	cd ${GCC_BUILD_DIR}/objdir && make install


${DEPS_DIR}/gcc7 : ${REPOS_DIR}/gcc7/
	cd $^ && ./contrib/download_prerequisites
	mkdir -p ${GCC_BUILD_DIR}/objdir
	rm -rf ${GCC_BUILD_DIR}/objdir/*
	cd  ${GCC_BUILD_DIR}/objdir && $</configure --prefix=$@ --enable-languages=c,c++ --disable-multilib --disable-libsanitizer
	+cd ${GCC_BUILD_DIR}/objdir && make
	mkdir -p $@
	cd ${GCC_BUILD_DIR}/objdir && make install


gcc5:
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi

gcc7:
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi

.PHONY: gcc5 gcc7
