BRISK_INCLUDE_DIR=${DEPS_DIR}/brisk/include
BRISK_DIR=${DEPS_DIR}/brisk/lib/CMake/brisk/

${REPOS_DIR}/brisk :
	mkdir ${REPOS_DIR} -p
	rm $@ -rf
	mkdir $@ -p
	cd $@ ; wget --no-check-certificate https://www.doc.ic.ac.uk/~sleutene/software/brisk-2.0.5.zip
	cd $@ ; unzip *.zip ; rm brisk-2.0.5.zip;
	sed -i.bak "s/[#]include [<]algo/#include<functional>\\n#include<algo/" ${REPOS_DIR}/brisk/src/brisk-feature-detector.cc

${DEPS_DIR}/brisk : ${REPOS_DIR}/brisk opencv
	cd ${REPOS_DIR}/brisk && OpenCV_DIR=${OPENCV_DIR} cmake . "-DCMAKE_INSTALL_PREFIX:PATH=$@" 
	cd ${REPOS_DIR}/brisk && make -j2
	mkdir -p $@
	cd ${REPOS_DIR}/brisk && make install

brisk :
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi
.PHONY: brisk
