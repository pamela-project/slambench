OPENGV_INCLUDE_DIR=${DEPS_DIR}/opengv/include
OPENGV_LIBRARY=${DEPS_DIR}/opengv/lib/libopengv.a

${REPOS_DIR}/opengv :
	mkdir -p ${REPOS_DIR}
	rm -rf ${REPOS_DIR}/opengv	
	cd ${REPOS_DIR} ; git clone "https://github.com/laurentkneip/opengv" opengv
	cd $@ ; git checkout cc32b16281aa6eab67cb28a61cf87a2a5c2b0961

${DEPS_DIR}/opengv : ${REPOS_DIR}/opengv eigen3
	mkdir ${REPOS_DIR}/opengv/build -p
	rm ${REPOS_DIR}/opengv/build/* -rf
	cd ${REPOS_DIR}/opengv/build && cmake .. "-DCMAKE_INSTALL_PREFIX:PATH=$@"  -DCMAKE_CXX_FLAGS="-w -O3 -std=c++11" -DEIGEN_VERSION_OK=3 -DEIGEN_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}
	+cd ${REPOS_DIR}/opengv/build && make
	mkdir -p $@
	cd ${REPOS_DIR}/opengv/build && make install


ifdef EIGEN3_INCLUDE_DIR
	opengv :
		+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi
else
	opengv :
		@echo "*** Error eigen not defined or not found"
		@echo "*** EIGEN_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}"
		@exit 1
endif

.PHONY: opengv
