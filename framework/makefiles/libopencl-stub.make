dependencies/libopencl-stub : 


ANDROID_OPENCL_ROOT_DIR=${ANDROID_DEPS_DIR}/libopencl-stub/
ANDROID_OPENCL_INCLUDE_DIR=${ANDROID_DEPS_DIR}/libopencl-stub/include
ANDROID_OPENCL_LIBRARY=${ANDROID_DEPS_DIR}/libopencl-stub/obj/local/armeabi-v7a/libOpenCL.a


PATCH_DIR=${ROOT_DIR}/framework/patchs/

${REPOS_DIR}/libopencl-stub :
	mkdir -p ${REPOS_DIR}/
	git clone https://github.com/krrishnarraj/libopencl-stub.git $@
	cd $@  && git checkout b4f84459e3a3a14d6a18b5dabe0a6ae9cbef709e
	cd $@  && git apply ${PATCH_DIR}/libopencl-stub_SLAMBench.patch

	
${ANDROID_DEPS_DIR}/libopencl-stub: ${REPOS_DIR}/libopencl-stub
	mkdir -p ${ANDROID_DEPS_DIR}/libopencl-stub
	rm -rf ${ANDROID_DEPS_DIR}/libopencl-stub/*
	cd ${REPOS_DIR}/libopencl-stub && ${ANDROID_NDK}/ndk-build NDK_PROJECT_PATH=. APP_BUILD_SCRIPT=Android.mk NDK_APPLICATION_MK=Application.mk 
	cp ${REPOS_DIR}/libopencl-stub/include ${ANDROID_DEPS_DIR}/libopencl-stub -rf
	cp ${REPOS_DIR}/libopencl-stub/obj ${ANDROID_DEPS_DIR}/libopencl-stub  -rf

android-libopencl-stub: ${ANDROID_DEPS_DIR}/libopencl-stub


.PHONY: android-libopencl-stub
	
	
	