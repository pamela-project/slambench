

CVD_INCLUDE_DIR=${DEPS_DIR}/cvd/include
CVD_LIBRARY=${DEPS_DIR}/cvd/lib/libcvd.so


${REPOS_DIR}/cvd :
	mkdir -p ${REPOS_DIR}
	rm -rf ${REPOS_DIR}/cvd
	git clone "https://github.com/edrosten/libcvd" ${REPOS_DIR}/cvd
	cd ${REPOS_DIR}/cvd && git checkout d190474150d4695e4c957863c5121c7eb79615d9

${DEPS_DIR}/cvd: ${REPOS_DIR}/cvd toon
	cd ${REPOS_DIR}/cvd && ./configure --prefix=$@ --without-ffmpeg  --without-v4l1buffer --without-dc1394v1 --without-dc1394v2 CPPFLAGS="-I${DEPS_DIR}/toon/include "
	mkdir -p $@
	+cd ${REPOS_DIR}/cvd && make install


android-cvd: ${ANDROID_DEPS_DIR}/cvd

cvd :
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi
.PHONY: cvd android-cvd
