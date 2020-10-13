SUITE_SPARSE_ROOT=${DEPS_DIR}/suitesparse

${REPOS_DIR}/suitesparse :
	mkdir ${REPOS_DIR}/ -p
	rm ${REPOS_DIR}/suitesparse -rf
	cd ${REPOS_DIR}/ && git clone "https://github.com/jluttine/suitesparse.git" 
	cd ${REPOS_DIR}/suitesparse && git checkout v4.3.1


${REPOS_DIR}/suitesparse/CMakeLists.txt : ${REPOS_DIR}/suitesparse
	echo -e "cmake_minimum_required(VERSION 2.8)\nproject(suitesparse)\ninclude_directories(SuiteSparse_config/)\ninclude_directories(CXSparse/Include)\ninclude_directories(\"${ANDROID_NDK}/platforms/android-21/arch-arm/usr/include\")\nFILE(GLOB cxsparse_source CXSparse/Source/*.c)\nadd_library(cxsparse_di  \$${cxsparse_source} )\nadd_library(cxsparse_dl  \$${cxsparse_source} )\nadd_library(cxsparse_ci  \$${cxsparse_source} )\nadd_library(cxsparse_cl  \$${cxsparse_source} )\nSET_TARGET_PROPERTIES(cxsparse_di  PROPERTIES COMPILE_FLAG \"\"  )\nSET_TARGET_PROPERTIES(cxsparse_dl  PROPERTIES COMPILE_FLAG \" -DCS_LONG\"  )\nSET_TARGET_PROPERTIES(cxsparse_ci  PROPERTIES COMPILE_FLAG \"-DCS_COMPLEX\"  )\nSET_TARGET_PROPERTIES(cxsparse_cl  PROPERTIES COMPILE_FLAG \" -DCS_LONG -DCS_COMPLEX\"  )\nINSTALL(TARGETS cxsparse_di  cxsparse_dl  cxsparse_ci  cxsparse_cl \n RUNTIME DESTINATION \$${CMAKE_INSTALL_PREFIX}/bin\n LIBRARY DESTINATION \$${CMAKE_INSTALL_PREFIX}/lib\n ARCHIVE DESTINATION \$${CMAKE_INSTALL_PREFIX}/lib\n)\n" > ${REPOS_DIR}/suitesparse/CMakeLists.txt

## Bruno : TODO : I'm not happy with this build method ... I'd rather like having a full cmake version of it...
## Harry: Suitesparse is not safe to parallel build :-(
${DEPS_DIR}/suitesparse : ${REPOS_DIR}/suitesparse ${REPOS_DIR}/suitesparse/CMakeLists.txt 
	cd ${REPOS_DIR}/suitesparse/ && make
	rm $@ -rf
	mkdir $@/include -p
	mkdir $@/lib -p
	+cd ${REPOS_DIR}/suitesparse/ && make install INSTALL_LIB=$@/lib INSTALL_INCLUDE=$@/include 

${ANDROID_DEPS_DIR}/suitesparse : ${REPOS_DIR}/suitesparse ${REPOS_DIR}/suitesparse/CMakeLists.txt   ${REPOS_DIR}/android-cmake
	cd ${REPOS_DIR}/suitesparse && mkdir build_dir -p
	cd ${REPOS_DIR}/suitesparse && rm build_dir/* -rf
	cd ${REPOS_DIR}/suitesparse/build_dir && cmake  -DBUILD_SHARED_LIBS=OFF  -DCMAKE_TOOLCHAIN_FILE=${REPOS_DIR}/android-cmake/android.toolchain.cmake -DANDROID_NDK=${ANDROID_NDK} -DCMAKE_BUILD_TYPE=Release -DANDROID_ABI="armeabi-v7a with NEON"  -D CMAKE_INSTALL_PREFIX=$(ANDROID_DEPS_DIR)    ..
	+cd ${REPOS_DIR}/suitesparse/build_dir && make
	cd ${REPOS_DIR}/suitesparse/build_dir && make install



suitesparse : 
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi
android-suitesparse : ${ANDROID_DEPS_DIR}/suitesparse

.PHONY: suitesparse android-suitesparse
