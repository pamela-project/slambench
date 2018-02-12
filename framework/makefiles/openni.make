
########################################## OPENNI 1.5 PART #################################################################

OPENNI_INCLUDE_DIR=${DEPS_DIR}/openni15/Include
OPENNI_LIBRARY=${DEPS_DIR}/openni15/Lib/libOpenNI.so

OPENNI_COMMIT=unstable
SENSOR_COMMIT=unstable

${REPOS_DIR}/libusb :
	rm -rf $@
	git clone git://git.libusb.org/libusb.git $@

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
	cd ${REPOS_DIR} ; git clone https://github.com/OpenNI/OpenNI.git $@ && cd $@ && git checkout ${OPENNI_COMMIT}
	sed -i".bak" "s/equivalent/glh_equivalent/g" ${REPOS_DIR}/openni15/Samples/NiViewer/glh/glh_linear.h


${REPOS_DIR}/SensorKinect :
	mkdir -p ${REPOS_DIR}
	rm -rf ${REPOS_DIR}/SensorKinect	
	cd ${REPOS_DIR} ; git clone https://github.com/avin2/SensorKinect.git $@ && cd $@ && git checkout ${SENSOR_COMMIT}


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
	git clone https://github.com/occipital/OpenNI2/ $@ 
	cd $@ && sed -i.bak "s/.javaDocExe, .-d., .java../[javaDocExe, '-d', 'java', '-Xdoclint:none']/" Source/Documentation/Runme.py

${REPOS_DIR}/freenect :
	mkdir -p ${REPOS_DIR}
	rm -rf ${REPOS_DIR}/freenect	
	git clone https://github.com/OpenKinect/libfreenect.git $@


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
