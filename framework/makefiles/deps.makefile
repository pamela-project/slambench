ROOT_DIR=$(shell pwd)
DEPS_DIR=$(ROOT_DIR)/deps
REPOS_DIR=$(DEPS_DIR)/repos
DEPS_BUILD_DIR=$(DEPS_DIR)/build
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

ifneq ("$(wildcard ${GCC7_COMPILER})","")
DEPS_ARGS+= -DCUDA_HOST_COMPILER=${GCC7_COMPILER}
endif

ifneq ("$(wildcard ${GCC5_COMPILER})","")
DEPS_ARGS+= -DCUDA_HOST_COMPILER=${GCC5_COMPILER}
endif

ifneq ("$(wildcard ${CERES_DIR})","")
DEPS_ARGS+= -DCERES_DIR=${CERES_DIR}
endif

ifneq ("$(wildcard ${Sophus_DIR})","")
DEPS_ARGS+= -DSophus_DIR=${Sophus_DIR}
endif

ifneq ("$(wildcard ${Sophus_INCLUDE_DIRS})","")
DEPS_ARGS+= -DSophus_INCLUDE_DIR=${Sophus_INCLUDE_DIR}
DEPS_ARGS+= -DSophus_INCLUDE_DIRS=${Sophus_INCLUDE_DIRS}
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

#### Compilation targets
####################################
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
	+make Sophus

.PHONY: deps
