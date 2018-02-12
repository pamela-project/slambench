
CUDA_HOST_COMPILER=${DEPS_DIR}/gcc_cuda/bin/c++


${REPOS_DIR}/gcc_cuda :
	mkdir -p ${REPOS_DIR}
	rm -rf ${REPOS_DIR}/gcc_cuda
	cd ${REPOS_DIR} && git clone git://gcc.gnu.org/git/gcc.git gcc_cuda
	cd ${REPOS_DIR}/gcc_cuda && git checkout gcc-5_3_0-release
	cd ${REPOS_DIR}/gcc_cuda && git cherry-pick ec1cc0263f156f70693a62cf17b254a0029f4852 || true


${DEPS_DIR}/gcc_cuda : ${REPOS_DIR}/gcc_cuda
	cd $^ && ./contrib/download_prerequisites
	mkdir -p $^/objdir
	cd $^/objdir && ../configure --prefix=$@ --enable-languages=c,c++ --disable-multilib
	+cd $^/objdir && make
	mkdir -p $@
	cd $^/objdir && make install

gcc_cuda:
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi

.PHONY: gcc_cuda
