TOON_INCLUDE_DIR=${DEPS_DIR}/toon/include
ANDROID_TOON_INCLUDE_DIR=${ANDROID_DEPS_DIR}/toon/include

${REPOS_DIR}/TooN :
	mkdir ${REPOS_DIR} -p
	rm ${REPOS_DIR}/TooN -rf
	git clone "https://github.com/edrosten/TooN.git" ${REPOS_DIR}/TooN
	cd ${REPOS_DIR}/TooN && git checkout 92241416d2a4874fd2334e08a5d417dfea6a1a3f

${DEPS_DIR}/toon: ${REPOS_DIR}/TooN
	cd ${REPOS_DIR}/TooN && ./configure --prefix=$@
	mkdir -p $@
	+cd ${REPOS_DIR}/TooN && make install

${ANDROID_DEPS_DIR}/toon: ${REPOS_DIR}/TooN
	mkdir -p $@
	cd ${REPOS_DIR}/TooN && ./configure --prefix=$@  --disable-lapack --enable-typeof=decltype
	+cd ${REPOS_DIR}/TooN && make install


android-toon: ${ANDROID_DEPS_DIR}/toon

toon : 
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi

.PHONY: toon android-toon
