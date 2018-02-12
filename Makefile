ECHO=/bin/echo

####################################
#### SLAMBENCH  INFOS (DEFAULT) ####
####################################

infos :
	@${ECHO} -e "\n*** SLAMBench is an evaluation framework for SLAM algorithms. *** \n\n\
	  * Use-cases are provided :"
	@for f in `ls -F benchmarks|grep /`;do ${ECHO} "    -  $$f"; done
	@${ECHO} -e "\n\
	  * To compile the SLAMBench framework you just need to type :\n\
	     - make slambench\n\n\
	  * To compile the use-cases as well :\n\
	     - make slambench APPS=\"kfusion;lsdslam\"\n\n\
	Please note several dependencies are needed to compile SLAMBench and its use-cases.\n\
	If you may have already them installed on your system,\n\
	we propose you to download and install them automatically using the following command :\n\
	     - make deps\n\
	"

####################################
#### SLAMBENCH BUILD            ####
####################################

ROOT_DIR=$(shell pwd)
DEPS_DIR=$(ROOT_DIR)/deps
REPOS_DIR=$(DEPS_DIR)/repos
DEPS_ARGS=
DEPS_ENV=

include framework/makefiles/*.make


ifneq ("$(wildcard ${OPENNI2_LIBRARY})","")
DEPS_ARGS+= -DOPENNI2_LIBRARY=${OPENNI2_LIBRARY}
endif

ifneq ("$(wildcard ${OPENNI2_INCLUDE_PATH})","")
DEPS_ARGS+= -DOPENNI2_INCLUDE_PATH=${OPENNI2_INCLUDE_PATH}
endif


ifneq ("$(wildcard ${CUDA_TOOLKIT_ROOT_DIR})","")
DEPS_ARGS+= -DCUDA_TOOLKIT_ROOT_DIR=${CUDA_TOOLKIT_ROOT_DIR}
endif
ifneq ("$(wildcard ${CUDA_HOST_COMPILER})","")
DEPS_ARGS+= -DCUDA_HOST_COMPILER=${CUDA_HOST_COMPILER}
endif

ifneq ("$(wildcard ${CERES_DIR})","")
DEPS_ARGS+= -DCERES_DIR=${CERES_DIR}
endif

ifneq ("$(wildcard ${GVARS_INCLUDE_DIR})","")
DEPS_ARGS+= -DGVARS_INCLUDE_DIR=${GVARS_INCLUDE_DIR}
endif


ifneq ("$(wildcard ${GVARS_LIBRARY})","")
DEPS_ARGS+= -DGVARS_LIBRARY=${GVARS_LIBRARY}
endif


ifneq ("$(wildcard ${CVD_INCLUDE_DIR})","")
DEPS_ARGS+= -DCVD_INCLUDE_DIR=${CVD_INCLUDE_DIR}
endif


ifneq ("$(wildcard ${CVD_LIBRARY})","")
DEPS_ARGS+= -DCVD_LIBRARY=${CVD_LIBRARY}
endif

ifneq ("$(wildcard ${OPENGV_INCLUDE_DIR})","")
DEPS_ARGS+= -DOPENGV_INCLUDE_DIR=${OPENGV_INCLUDE_DIR}
endif

ifneq ("$(wildcard ${OPENGV_LIBRARY})","")
DEPS_ARGS+= -DOPENGV_LIBRARY=${OPENGV_LIBRARY}
endif

ifneq ("$(wildcard ${BRISK_INCLUDE_DIR})","")
DEPS_ARGS+= -DBRISK_INCLUDE_PATH=${BRISK_INCLUDE_DIR}
endif

ifneq ("$(wildcard ${PANGOLIN_DIR})","")
DEPS_ENV+= Pangolin_DIR=${PANGOLIN_DIR}
endif

ifneq ("$(wildcard ${EIGEN3_INCLUDE_DIR})","")
DEPS_ARGS+= -DEIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR} -DEIGEN3_VERSION_OK=ON 
DEPS_ARGS+= -DEIGEN_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR} -DEIGEN_VERSION_OK=ON 
endif

ifneq ("$(wildcard ${TOON_INCLUDE_DIR})","")
DEPS_ARGS+= -DTOON_INCLUDE_PATH=${TOON_INCLUDE_DIR}
endif

ifneq ("$(wildcard ${OPENCV_DIR})","")
DEPS_ENV+= OpenCV_DIR=${OPENCV_DIR}
endif	

ifneq ("$(wildcard ${BRISK_DIR})","")
DEPS_ENV+= brisk_DIR=${BRISK_DIR}/
endif

ifneq ("$(wildcard ${PCL_DIR})","")
DEPS_ENV+= PCL_DIR=${PCL_DIR}/
endif

ifneq ("$(wildcard ${G2O_DIR})","")
DEPS_ENV+= G2O_ROOT=${G2O_DIR}/
endif

ifneq ("$(wildcard ${CUDA_INC_PATH})","")
DEPS_ENV+= CUDA_INC_PATH=${CUDA_INC_PATH}/
endif

ifneq ("$(wildcard ${SUITE_SPARSE_ROOT})","")
	DEPS_ARGS+= -DSUITE_SPARSE_ROOT=${SUITE_SPARSE_ROOT}
endif

ifneq ("$(wildcard ${FREEIMAGE_INCLUDE_PATH})","")
	DEPS_ARGS+= -DFreeImage_INCLUDE_PATH="${FREEIMAGE_INCLUDE_PATH}"
	DEPS_ARGS+= -DFreeImage_DYNAMIC_LIBRARY="${FREEIMAGE_DYNAMIC_LIBRARY}"
endif

deps : 
	+make brisk
	+make ceres
	+make cvd
	+make eigen3
	+make flann
	+make freeimage
	+make g2o
	+make gvars
	+make opencv
	+make opengv
	+make opentuner
	+make pangolin
	+make pcl
	+make suitesparse
	+make toon


slambench :
	mkdir -p build/
	mkdir -p install/
	${DEPS_ENV} cmake  -Bbuild -H.  -DAPPS="${APPS}" ${DEPS_ARGS} -DCMAKE_INSTALL_PREFIX:PATH=${ROOT_DIR}/install
	$(MAKE) -C build $(MFLAGS) 


####################################
#### SLAMBENCH ANDROID BUILD    ####
####################################


ROOT_DIR=$(shell pwd)
ANDROID_NDK=~/.local/Android/android-ndk-r13b
ANDROID_TOOLCHAIN_FILE=${ANDROID_NDK}/build/cmake/android.toolchain.cmake
ANDROID_DEPS_DIR=$(ROOT_DIR)/android-deps
ANDROID_DEPS_ARGS=
ANDROID_DEPS_ENV=
ANDROID_CMAKE_PARAMETERS=

# Select the toolchain from Android NDK

ANDROID_CMAKE_PARAMETERS+=	-DCMAKE_TOOLCHAIN_FILE=${ANDROID_TOOLCHAIN_FILE} 

# Known parameters from the cmake page of NDK

ANDROID_CMAKE_PARAMETERS+=	-DANDROID_TOOLCHAIN=gcc                               # clang or gcc
ANDROID_CMAKE_PARAMETERS+=	-DANDROID_PLATFORM=android-18                         # android-18 , ..., android-21
ANDROID_CMAKE_PARAMETERS+=	-DANDROID_STL="gnustl_static"                            # gnustl_static, ...
ANDROID_CMAKE_PARAMETERS+=	-DANDROID_CPP_FEATURES="exceptions;rtti"  
ANDROID_CMAKE_PARAMETERS+=	-DANDROID_ALLOW_UNDEFINED_SYMBOLS=FALSE
ANDROID_CMAKE_PARAMETERS+=	-DANDROID_ARM_MODE=thumb
ANDROID_CMAKE_PARAMETERS+=	-DANDROID_ARM_NEON=FALSE
ANDROID_CMAKE_PARAMETERS+=	-DANDROID_DISABLE_NO_EXECUTE=FALSE
ANDROID_CMAKE_PARAMETERS+=	-DANDROID_DISABLE_RELRO=FALSE
ANDROID_CMAKE_PARAMETERS+=	-DANDROID_DISABLE_FORMAT_STRING_CHECKS=FALSE                   

# Extra parameters

ANDROID_CMAKE_PARAMETERS+=	-DANDROID_ABI="armeabi-v7a"                                     
ANDROID_CMAKE_PARAMETERS+=	-DCMAKE_BUILD_TYPE=Release    
ANDROID_CMAKE_PARAMETERS+=	-DANDROID_NDK=${ANDROID_NDK}        


ifneq ("$(wildcard ${ANDROID_TOON_INCLUDE_DIR})","")
	ANDROID_DEPS_ARGS+= -DTOON_INCLUDE_PATH=${ANDROID_TOON_INCLUDE_DIR}
endif

ifneq ("$(wildcard ${ANDROID_EIGEN3_INCLUDE_DIR})","")
	ANDROID_DEPS_ARGS+= -DEIGEN3_INCLUDE_DIR=${ANDROID_EIGEN3_INCLUDE_DIR} -DEIGEN3_VERSION_OK=ON 
	ANDROID_DEPS_ARGS+= -DEIGEN_INCLUDE_DIR=${ANDROID_EIGEN3_INCLUDE_DIR} -DEIGEN_VERSION_OK=ON 
	ANDROID_DEPS_ARGS+= -DEIGEN_INCLUDE_DIRS=${ANDROID_EIGEN3_INCLUDE_DIR} -DEIGEN_VERSION_OK=ON 
endif

ifneq ("$(wildcard ${ANDROID_OPENCL_INCLUDE_DIR})","")
	ANDROID_DEPS_ARGS+= -DOPENCL_ROOT_DIR=${ANDROID_OPENCL_ROOT_DIR} 
	ANDROID_DEPS_ARGS+= -DOPENCL_INCLUDE_DIR=${ANDROID_OPENCL_INCLUDE_DIR} 
	ANDROID_DEPS_ARGS+= -DOPENCL_LIBRARY=${ANDROID_OPENCL_LIBRARY} 
endif


android-deps : 
	+make android-cmake 
	+make android-toon 
	+make android-eigen3 
	+make android-suitesparse 
	+make android-opencv 
	+make android-g2o


android-slambench :
	mkdir -p android-build/
	mkdir -p android-install/
	${ANDROID_DEPS_ENV} cmake ${ANDROID_CMAKE_PARAMETERS}	-Bandroid-build -H.  -DAPPS="${APPS}" ${ANDROID_DEPS_ARGS} -DCMAKE_INSTALL_PREFIX:PATH=${ROOT_DIR}/android-install
	$(MAKE) -C android-build $(MFLAGS)
	$(MAKE) -C android-build install $(MFLAGS) 

####################################
####    BUILD/CLEAN TOOL        ####
####################################


#### BUILDALL TOOLS ####

buildall: 
	+make slambench APPS=all 


#### CLEAN/CLEANALL TOOLS ####
clean :
	rm -rf build install android-build android-install *.log framework/scripts/slambench/*.pyc

cleanall : clean
	rm -rf repos deps
	rm -f *.log 


