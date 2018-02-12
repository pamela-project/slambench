


${REPOS_DIR}/gcc7 :
	mkdir -p ${REPOS_DIR}
	rm -rf ${REPOS_DIR}/gcc7
	cd ${REPOS_DIR} && git clone git://gcc.gnu.org/git/gcc.git gcc7 
	cd ${REPOS_DIR}/gcc7 && git checkout gcc-7_1_0-release


${DEPS_DIR}/gcc7 : ${REPOS_DIR}/gcc7
	cd $^ && ./contrib/download_prerequisites
	mkdir -p $^/objdir
	cd $^/objdir && ../configure --prefix=$@ --enable-languages=c,c++ --disable-multilib
	+cd $^/objdir && make
	mkdir -p $@
	cd $^/objdir && make install

gcc7:
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi

.PHONY: gcc7
