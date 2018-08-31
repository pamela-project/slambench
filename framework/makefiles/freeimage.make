
FREEIMAGE_INCLUDE_PATH=${DEPS_DIR}/freeimage/include
FREEIMAGE_DYNAMIC_LIBRARY=${DEPS_DIR}/freeimage/lib/libfreeimage.so
FREEIMAGE_URL=http://downloads.sourceforge.net/freeimage/FreeImage3180.zip

${REPOS_DIR}/FreeImage :
	mkdir ${REPOS_DIR} -p
	rm ${REPOS_DIR}/FreeImage -rf
	cd ${REPOS_DIR} && wget ${FREEIMAGE_URL} && unzip FreeImage3180.zip && rm FreeImage3180.zip
	#git clone https://github.com/mikesart/freeimage.git  ${REPOS_DIR}/FreeImage
	#cd ${REPOS_DIR}/FreeImage && git checkout d49fb3982c1cb7826bc10edaf5c0ac4d9104660f
#cd ${REPOS_DIR}/ && cvs -z3 -d":pserver:anonymous@freeimage.cvs.sourceforge.net:/cvsroot/freeimage" co  -P FreeImage
	sed -i.bak "s/-o root -g root//" ${REPOS_DIR}/FreeImage/Makefile.gnu
	sed -i.bak "s/-Wno-ctor-dtor-privacy/-w/" ${REPOS_DIR}/FreeImage/Makefile.gnu
	sed -i.bak "s/defined._ARM_./defined\(_ARM_\) or defined\(__arm__\)/"  ${REPOS_DIR}/FreeImage/Source/LibRawLite/libraw/libraw_types.h

${DEPS_DIR}/freeimage : ${REPOS_DIR}/FreeImage
	+cd ${REPOS_DIR}/FreeImage && make DESTDIR=$@ 
	mkdir -p $@
	cd ${REPOS_DIR}/FreeImage && make install DESTDIR=$@ INCDIR=$@/include INSTALLDIR=$@/lib




freeimage :
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi

.PHONY: freeimage 
