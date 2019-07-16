PANGOLIN_DIR=${DEPS_DIR}/pangolin/lib/cmake/Pangolin/

CURRENT_PANGOLIN_COMMIT=c2a6ef524401945b493f14f8b5b8aa76cc7d71a9 # (Mar 18)

## INFO: Previously we used 8b8b7b96adcf58ac2755dedd3f681fc512385af0 (Jan 17)

${REPOS_DIR}/pangolin :
	mkdir -p ${REPOS_DIR}
	rm -rf ${REPOS_DIR}/pangolin
	git clone "https://github.com/stevenlovegrove/Pangolin.git" ${REPOS_DIR}/pangolin
	cd ${REPOS_DIR}/pangolin && git checkout ${CURRENT_PANGOLIN_COMMIT}


${DEPS_DIR}/pangolin : ${REPOS_DIR}/pangolin  eigen3
	if [ ! -e $(EIGEN3_INCLUDE_DIR) ] ; \
	then \
		echo "ERROR: Pangolin requires EIGEN to be fully functional (make eigen).";\
		exit 1;\
	fi;
	cd ${REPOS_DIR}/pangolin && mkdir -p build && cd build && cmake .. -DBUILD_PANGOLIN_VIDEO=OFF  "-DCMAKE_INSTALL_PREFIX:PATH=$@"  -DEIGEN_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}
	+cd ${REPOS_DIR}/pangolin/build && make
	mkdir -p $@
	cd ${REPOS_DIR}/pangolin/build && make install

pangolin : 
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else if cat ${DEPS_DIR}/pangolin/include/pangolin/config.h |grep -q "define HAVE_EIGEN" ; then echo "$@ skipped."; else make ${DEPS_DIR}/$@ ; fi; fi
	@echo "CHECK PANGOLIN"
	@cat ${DEPS_DIR}/pangolin/include/pangolin/config.h |grep -q "define HAVE_EIGEN"
	@echo "PANGOLIN INSTALLED"

.PHONY: pangolin
