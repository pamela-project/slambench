FREEIMAGE_INCLUDE_PATH=${DEPS_DIR}/freeimage/include
FREEIMAGE_DYNAMIC_LIBRARY=${DEPS_DIR}/freeimage/lib/libfreeimage.so
FREEIMAGE_URL=http://downloads.sourceforge.net/freeimage/FreeImage3180.zip

${REPOS_DIR}/FreeImage :
	mkdir ${REPOS_DIR} -p
	rm ${REPOS_DIR}/FreeImage -rf
	cd ${REPOS_DIR} && wget ${FREEIMAGE_URL} && unzip FreeImage3180.zip && rm FreeImage3180.zip
	sed -i.bak "s/-o root -g root//" ${REPOS_DIR}/FreeImage/Makefile.gnu
	sed -i.bak "s/-Wno-ctor-dtor-privacy/-w/" ${REPOS_DIR}/FreeImage/Makefile.gnu
	sed -i.bak "s/defined._ARM_./defined\(_ARM_\) or defined\(__arm__\)/"  ${REPOS_DIR}/FreeImage/Source/LibRawLite/libraw/libraw_types.h
	cd ${REPOS_DIR}/FreeImage && git apply ${ROOT_DIR}/framework/patchs/freeimage.patch

${DEPS_DIR}/freeimage : ${REPOS_DIR}/FreeImage
	+cd ${REPOS_DIR}/FreeImage && make DESTDIR=$@ 
	mkdir -p $@
	cd ${REPOS_DIR}/FreeImage && make install DESTDIR=$@ INCDIR=$@/include INSTALLDIR=$@/lib

freeimage :
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi

.PHONY: freeimage 
