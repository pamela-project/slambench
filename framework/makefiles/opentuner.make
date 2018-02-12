
${REPOS_DIR}/opentuner :
	mkdir ${REPOS_DIR} -p
	rm $@ -rf
	git clone https://github.com/jansel/opentuner.git  $@ 

${DEPS_DIR}/opentuner : ${REPOS_DIR}/opentuner 
	mkdir -p $@
	cp -r ${REPOS_DIR}/opentuner/opentuner  ${DEPS_DIR}/opentuner/

opentuner : 
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi


.PHONY: opentuner
