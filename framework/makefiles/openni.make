########################################## COMMIT USED #################################################################
OPENNI2_REPOS=https://github.com/occipital/OpenNI2/
OPENNI2_COMMIT=1fce8edffab43c4a4cf201cff86f415b07a2d37f

OPENNI15_REPOS=https://github.com/OpenNI/OpenNI.git
OPENNI15_COMMIT=54e899c492f69aa8fa3e133fdd7d6b468f017b99

LIBUSB_REPOS=https://github.com/libusb/libusb.git
LIBUSB_COMMIT=8ddd8d994df6e367603266630bc2fe83b9cad868

SENSORK_REPOS=https://github.com/avin2/SensorKinect.git
SENSORK_COMMIT=15f1975d5e50d84ca06ff784f83f8b7836749a7b

FREENECT_REPOS=https://github.com/OpenKinect/libfreenect.git
FREENECT_COMMIT=83e57e1318cc64c9aabac481b9e330acc1914a23

########################################## OPENNI 1.5 PART #################################################################
OPENNI_INCLUDE_DIR=${DEPS_DIR}/openni15/Include
OPENNI_LIBRARY=${DEPS_DIR}/openni15/Lib/libOpenNI.so

${REPOS_DIR}/libusb :
	rm -rf $@
	git clone ${LIBUSB_REPOS} $@ && cd $@ && git checkout ${LIBUSB_COMMIT}

${DEPS_DIR}/libusb : ${REPOS_DIR}/libusb
	cd  ${REPOS_DIR}/libusb && ./autogen.sh  --prefix=$@
	cd  ${REPOS_DIR}/libusb && make
	mkdir -p ${DEPS_DIR}/libusb
	rm -rf ${DEPS_DIR}/libusb/*
	cd  ${REPOS_DIR}/libusb && make install

libusb :
	@+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi

${REPOS_DIR}/openni15 : 
	mkdir -p ${REPOS_DIR}
	rm -rf ${REPOS_DIR}/openni15	
	cd ${REPOS_DIR} ; git clone ${OPENNI15_REPOS} $@ && cd $@ && git checkout ${OPENNI15_COMMIT}
	sed -i".bak" "s/equivalent/glh_equivalent/g" ${REPOS_DIR}/openni15/Samples/NiViewer/glh/glh_linear.h


${REPOS_DIR}/SensorKinect :
	mkdir -p ${REPOS_DIR}
	rm -rf ${REPOS_DIR}/SensorKinect	
	cd ${REPOS_DIR} ; git clone ${SENSORK_REPOS} $@ && cd $@ && git checkout ${SENSORK_COMMIT}

${DEPS_DIR}/openni15 : ${REPOS_DIR}/openni15 ${DEPS_DIR}/libusb 
	chmod +x ${REPOS_DIR}/openni15/Platform/Linux/CreateRedist/RedistMaker
	cd ${REPOS_DIR}/openni15/Platform/Linux/CreateRedist/ && ./RedistMaker
	rm -rf ${DEPS_DIR}/openni15
	cd ${REPOS_DIR}/openni15 && tar xf ${REPOS_DIR}/openni15/Platform/Linux/CreateRedist/Final/*.tar.bz2
	mv ${REPOS_DIR}/openni15/OpenNI-Bin-*/  ${DEPS_DIR}/openni15
	cp ${REPOS_DIR}/openni15/Data/SamplesConfig.xml  ${DEPS_DIR}/openni15/

${DEPS_DIR}/SensorKinect : ${REPOS_DIR}/SensorKinect  ${REPOS_DIR}/openni15 
	cd $</Platform/Linux/CreateRedist && ./RedistMaker
	cp -rf $</Platform/Linux/Redist/Sensor-Bin-*/  $@/

openni15 :
	@+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi
	@echo "********************************************************************"
	@echo " Don't forget to run: cd ${DEPS_DIR}/$@  && sudo ./install.sh"
	@echo " To test it : cd ${DEPS_DIR}/openni15 && ./Samples/Bin/x64-Release/Sample-NiSimpleRead"
	@echo "********************************************************************"

SensorKinect :
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi
	@echo "********************************************************************"
	@echo " Don't forget to run: cd ${DEPS_DIR}/$@  && sudo ./install.sh"
	@echo " To test it : cd ${DEPS_DIR}/openni15 && ./Samples/Bin/x64-Release/Sample-NiSimpleRead"
	@echo "********************************************************************"

.PHONY: openni15  SensorKinect


########################################## OPENNI 2 PART #################################################################
# requires systemd-devel  doxygen

OPENNI2_INCLUDE=${DEPS_DIR}/openni2/OpenNI-Linux-x64-2.2/Include
OPENNI2_REDIST=${DEPS_DIR}/openni2/OpenNI-Linux-x64-2.2/Redist

OPENNI2_INCLUDE_PATH=${DEPS_DIR}/openni2/OpenNI-Linux-x64-2.2/Include
OPENNI2_LIBRARY=${DEPS_DIR}/openni2/OpenNI-Linux-x64-2.2/Redist/libOpenNI2.so

${REPOS_DIR}/openni2 :
	mkdir -p ${REPOS_DIR}
	rm -rf ${REPOS_DIR}/openni2	
	git clone ${OPENNI2_REPOS} $@ && cd $@ && git checkout  ${OPENNI2_COMMIT} 
	cd $@ && sed -i.bak "s/.javaDocExe, .-d., .java../[javaDocExe, '-d', 'java', '-Xdoclint:none']/" Source/Documentation/Runme.py

${REPOS_DIR}/freenect :
	mkdir -p ${REPOS_DIR}
	rm -rf ${REPOS_DIR}/freenect	
	git clone ${FREENECT_REPOS} $@ && cd $@ && git checkout  ${FREENECT_COMMIT}

${DEPS_DIR}/openni2 : ${REPOS_DIR}/openni2 ${REPOS_DIR}/freenect # Needs: libusbx-devel-1.0.21-1.fc25.x86_64
	cd ${REPOS_DIR}/openni2/Packaging && ./ReleaseVersion.py x64
	mkdir -p $@
	cd ${REPOS_DIR}/openni2/Packaging/Final && tar xf OpenNI-Linux-x64-2.2.tar.bz2 -C  $@
	cd ${REPOS_DIR}/freenect && mkdir -p build && rm build/* -rf
	cd ${REPOS_DIR}/freenect/build && cmake .. -DBUILD_OPENNI2_DRIVER=ON && make
	cp -L ${REPOS_DIR}/freenect/build/lib/OpenNI2-FreenectDriver/libFreenectDriver.so ${OPENNI2_REDIST}/OpenNI2/Drivers/

openni2 :
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi

.PHONY: openni2
