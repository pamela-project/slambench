OPENCV_DIR=${DEPS_DIR}/opencv/share/OpenCV/
ANDROID_OPENCV_DIR=${ANDROID_DEPS_DIR}/opencv/share/OpenCV/


${REPOS_DIR}/opencv :
	mkdir ${REPOS_DIR} -p
	rm ${REPOS_DIR}/opencv -rf
	git clone "https://github.com/Itseez/opencv.git" ${REPOS_DIR}/opencv
	cd ${REPOS_DIR}/opencv && git checkout 2c9547e && git cherry-pick bbe007159ad6b99e15f47171b7b6be7c892ca9fa


${DEPS_DIR}/opencv : ${REPOS_DIR}/opencv
	cd ${REPOS_DIR}/opencv && mkdir build_dir -p
	cd ${REPOS_DIR}/opencv && rm build_dir/* -rf
	cd ${REPOS_DIR}/opencv/build_dir && cmake -D CMAKE_BUILD_TYPE=RELEASE  -D CMAKE_INSTALL_PREFIX=$@    -DCMAKE_CXX_FLAGS="-Wno-error=address"             \
	-DWITH_GSTREAMER=OFF -DWITH_FFMPEG=OFF -DBUILD_PERF_TESTS=OFF  -D WITH_OPENCL=OFF -D BUILD_WITH_DEBUG_INFO=OFF  -D WITH_1394=OFF                \
	-D BUILD_TESTS=OFF  -D WITH_TBB=OFF  -D WITH_V4L=OFF  -D WITH_OPENGL=OFF -D BUILD_opencv_nonfree=1 -D BUILD_opencv_gpu=OFF    \
	  -D BUILD_opencv_java=OFF -D WITH_CUDA=OFF -DWITH_GTK=ON   -D BUILD_opencv_ml=ON  -D BUILD_opencv_videostab=OFF             \
	   -D BUILD_opencv_ts=OFF    -D BUILD_opencv_photo=ON  -D BUILD_opencv_video=ON -D BUILD_opencv_stitching=OFF -D BUILD_opencv_contrib=OFF -DENABLE_PRECOMPILED_HEADERS=OFF .. > ${REPOS_DIR}/opencv/build_dir/opencv_cmake.log
	cat ${REPOS_DIR}/opencv/build_dir/opencv_cmake.log
	cat ${REPOS_DIR}/opencv/build_dir/opencv_cmake.log | grep -E "     GTK[+] 2[.]x:                    YES"
	cat ${REPOS_DIR}/opencv/build_dir/opencv_cmake.log | grep -E "     To be built:                 .*nonfree.*"
	+cd ${REPOS_DIR}/opencv/build_dir && make
	mkdir -p $@
	cd ${REPOS_DIR}/opencv/build_dir && make install
	sed -i.bak "s/.*contrib.*//" $@/include/opencv2/opencv.hpp # FIX contrib.hpp bug

$(ANDROID_DEPS_DIR)/opencv : ${REPOS_DIR}/opencv  ${REPOS_DIR}/android-cmake
	cd ${REPOS_DIR}/opencv && mkdir build_dir -p
	cd ${REPOS_DIR}/opencv && rm build_dir/* -rf
	cd ${REPOS_DIR}/opencv/build_dir && cmake  -DBUILD_SHARED_LIBS=OFF  -DCMAKE_TOOLCHAIN_FILE=${REPOS_DIR}/android-cmake/android.toolchain.cmake -DANDROID_NDK=${ANDROID_NDK} -DCMAKE_BUILD_TYPE=Release -DANDROID_ABI="armeabi-v7a with NEON"   -DWITH_GSTREAMER=OFF -DBUILD_PERF_TESTS=OFF  -D WITH_OPENCL=OFF -D CMAKE_INSTALL_PREFIX=$(ANDROID_DEPS_DIR) -D BUILD_WITH_DEBUG_INFO=OFF  -D WITH_1394=OFF -D BUILD_TESTS=OFF  -D WITH_TBB=OFF  -D WITH_V4L=OFF  -D WITH_OPENGL=OFF -D BUILD_opencv_nonfree=1 -D BUILD_opencv_gpu=OFF -D BUILD_opencv_java=OFF -D WITH_CUDA=OFF -DWITH_GTK=OFF  -D BUILD_opencv_videostab=OFF  -D BUILD_opencv_ts=OFF  -D BUILD_opencv_stitching=OFF   .. 
	+cd ${REPOS_DIR}/opencv/build_dir && make
	cd ${REPOS_DIR}/opencv/build_dir && make install


opencv : 	
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi


android-opencv : ${ANDROID_DEPS_DIR}/opencv

.PHONY: opencv android-opencv
