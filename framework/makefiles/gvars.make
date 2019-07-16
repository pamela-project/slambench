

GVARS_INCLUDE_DIR=${DEPS_DIR}/gvars/include
GVARS_LIBRARY=${DEPS_DIR}/gvars/lib/libGVars3.so


${REPOS_DIR}/gvars :
	mkdir -p ${REPOS_DIR} 
	rm -rf ${REPOS_DIR}/gvars 
	git clone "https://github.com/edrosten/gvars" ${REPOS_DIR}/gvars
	cd ${REPOS_DIR}/gvars && git checkout fc58c500c9d8f8713fb87a98cf7fb6be1db3295f


${DEPS_DIR}/gvars: ${REPOS_DIR}/gvars toon
	cd ${REPOS_DIR}/gvars && ./configure --prefix=$@ CPPFLAGS="-I${DEPS_DIR}/toon/include -fPIC"
	+cd ${REPOS_DIR}/gvars && make 
	mkdir -p $@
	cd ${REPOS_DIR}/gvars && make install


gvars : 
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi

.PHONY: gvars 
