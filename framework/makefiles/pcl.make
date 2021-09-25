ifndef DEPS_DIR
$(error DEPS_DIR is not set)
endif

ifndef REPOS_DIR
$(error REPOS_DIR is not set)
endif

ifndef DEPS_BUILD_DIR
$(error DEPS_BUILD_DIR is not set)
endif

PCL_DIR=${DEPS_DIR}/pcl/share/pcl-1.8
ANDROID_PCL_DIR=${ANDROID_DEPS_DIR}/pcl/share/pcl-1.8

# DOWNLOADS #

${REPOS_DIR}/pcl :
	mkdir ${REPOS_DIR} -p
	rm ${REPOS_DIR}/pcl -rf
	git clone "https://github.com/PointCloudLibrary/pcl.git" ${REPOS_DIR}/pcl
	cd $@ && git checkout 6fb1b65d3099a915255b070269b1ac78ed384921
# BUILD #
${DEPS_DIR}/pcl : ${REPOS_DIR}/pcl eigen3 flann
	if [ ! -e $(EIGEN3_INCLUDE_DIR) ] ; \
	then \
		echo "ERROR: PCL requires EIGEN to be fully functional (make eigen).";\
		exit 1;\
	fi;
	mkdir ${DEPS_BUILD_DIR}/pcl/ -p
	rm ${DEPS_BUILD_DIR}/pcl/* -rf
	sed -i.bak "s/\(find_package.*\) mpi/\1/" ${REPOS_DIR}/pcl/cmake/pcl_find_boost.cmake # PCL should not need MPI !
	cd ${DEPS_BUILD_DIR}/pcl/ && cmake ${REPOS_DIR}/pcl \
	-DWITH_LIBUSB=FALSE \
	-DWITH_OPENNI=FALSE \
	-DWITH_OPENNI2=FALSE \
	-DWITH_FZAPI=FALSE \
	-DWITH_ENSENSO=FALSE \
	-DWITH_DAVIDSDK=FALSE \
	-DWITH_DSSDK=FALSE \
	-DWITH_RSSDK=FALSE \
	-DWITH_VTK=FALSE \
	-DWITH_PCAP=FALSE \
	-DBUILD_common=TRUE                \
	-DBUILD_octree=TRUE               \
	-DBUILD_io=TRUE                   \
	-DBUILD_kdtree=TRUE               \
	-DBUILD_search=TRUE               \
	-DBUILD_sample_consensus=TRUE     \
	-DBUILD_filters=TRUE              \
	-DBUILD_2d=TRUE                   \
	-DBUILD_geometry=TRUE             \
	-DBUILD_features=TRUE             \
	-DBUILD_ml=TRUE                   \
	-DBUILD_segmentation=FALSE         \
	-DBUILD_surface=TRUE              \
	-DBUILD_outofcore=FALSE              \
	-DBUILD_examples=FALSE            \
	-DBUILD_simulation=FALSE            \
	-DBUILD_registration=TRUE          \
	-DBUILD_keypoints=TRUE            \
	-DBUILD_tracking=FALSE             \
	-DBUILD_recognition=FALSE          \
	-DBUILD_stereo=FALSE               \
	-DBUILD_visualization=FALSE         \
	-DBUILD_tools=FALSE \
	"-DCMAKE_INSTALL_PREFIX:PATH=$@" \
	-DEIGEN_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR} \
	-DFLANN_LIBRARY=${FLANN_LIBRARY} -DCMAKE_BUILD_TYPE=RELEASE \
	-DFLANN_INCLUDE_DIR=${FLANN_INCLUDE_DIR}  > ${DEPS_BUILD_DIR}/pcl/build.log.tmp 2>&1
	if cat ${DEPS_BUILD_DIR}/pcl/build.log.tmp | grep "Requires external library" ; then echo "Error with deps of PCL." ; exit 1 ; fi
	+cd ${DEPS_BUILD_DIR}/pcl/ && make 
	mkdir -p $@
	cd ${DEPS_BUILD_DIR}/pcl/ && make install

pcl : 
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi

.PHONY: pcl

