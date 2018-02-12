PANGOLIN_DIR=${DEPS_DIR}/pangolin/lib/cmake/Pangolin/

${REPOS_DIR}/pangolin :
	mkdir ${REPOS_DIR} -p
	rm ${REPOS_DIR}/pangolin -rf
	git clone "https://github.com/stevenlovegrove/Pangolin.git" ${REPOS_DIR}/pangolin
	cd ${REPOS_DIR}/pangolin && git checkout 8b8b7b96adcf58ac2755dedd3f681fc512385af0
# git checkout b107252bf6dbb50b26597f5f2c2ca39c4412f72c
# git checkout master
# git checkout   021ed52ca8e355abf7cd2c783e12a316fc07218d
# git checkout e849e3b # original for LSDSLAM

${DEPS_DIR}/pangolin : ${REPOS_DIR}/pangolin  eigen3
	if [ ! -e $(EIGEN3_INCLUDE_DIR) ] ; \
	then \
		echo "ERROR: Pangolin requires EIGEN to be fully functional (make eigen).";\
		exit 1;\
	fi;
	cd ${REPOS_DIR}/pangolin && cmake . -DAVFORMAT_INCLUDE_DIR=\"\"  "-DCMAKE_INSTALL_PREFIX:PATH=$@" -DEIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR} -DEIGEN_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}
	+cd ${REPOS_DIR}/pangolin && make
	mkdir -p $@
	cd ${REPOS_DIR}/pangolin && make install

pangolin : 
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else if cat ${DEPS_DIR}/pangolin/include/pangolin/config.h |grep -q "define HAVE_EIGEN" ; then echo "$@ skipped."; else make ${DEPS_DIR}/$@ ; fi; fi
	@echo "CHECK PANGOLIN"
	@cat ${DEPS_DIR}/pangolin/include/pangolin/config.h |grep -q "define HAVE_EIGEN"
	@echo "PANGOLIN INSTALLED"

.PHONY: pangolin
