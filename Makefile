ECHO=/bin/echo
WGET:=wget
GET_REPLY:=read REPLY

ifeq ("${SBQUIET}","1")
GET_REPLY:=REPLY="y"
WGET:=wget -q
endif


####################################
#### SLAMBENCH  INFOS (DEFAULT) ####
####################################

infos :
	@${ECHO} -e "\n*** SLAMBench is an evaluation framework for SLAM algorithms. *** "
	@${ECHO} -e "\n\
	  (1) First, several dependencies are needed to compile SLAMBench and its use-cases.\n\
	      If you may have already them installed on your system,\n\
	      we propose you to download and install them automatically using the following command :\n\
	      - \033[1;32mmake deps\033[0m\n\n\
	      - \033[1;32mmake gcc_cuda (If you do not have a compatible toolchain for CUDA)\033[0m\n\n\
	  (2) Then, to compile the SLAMBench framework you just need to type :\n\
	      - \033[1;32mmake slambench\033[0m\n\n\
	  (3) To compile the use-cases as well :\n\
	      - \033[1;32mmake slambench APPS=\"kfusion;lsdslam\"\033[0m\n\n\
	      However these use-cases are not distributed with SLAMBench, you will have to download them using the command:\n\
	      - \033[1;32mmake usecases\033[0m\n\n\
	  (4) Finally to run SLAM system with a particular dataset we recommand to run:\n\
	      - \033[1;32mmake datasets\033[0m\n\n\
	"

####################################
#### SLAMBENCH BUILD            ####
####################################

ROOT_DIR=$(shell pwd)
DEPS_DIR=$(ROOT_DIR)/deps
REPOS_DIR=$(DEPS_DIR)/repos
DEPS_BUILD_DIR=$(DEPS_DIR)/build
DEPS_ARGS=
DEPS_ENV=


#### Where dependencies are
####################################



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


build/Makefile : framework/CMakeLists.txt
	mkdir -p build/
	cd build; ${DEPS_ENV} cmake -U -Bbuild -H.  -DAPPS="${APPS}"  ${DEPS_ARGS} -D"CMAKE_MODULE_PATH:PATH=${ROOT_DIR}/cmake_modules" ..


.PHONY: build/Makefile 

slambench: build/Makefile 
	$(MAKE) -C build $(MFLAGS)
	@echo ""
	@echo "================================================================================================================="
	@echo "The SLAMBench library should have been compiled, you will find binaries in build/bin, and libraries in build/lib."
	@echo ""
	@echo "The list of current binaries is: "
	@echo ""
	@echo "Loaders available: "
	@echo -n "  - build/bin/benchmark_loader:    " ; if [ -f build/bin/benchmark_loader ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (Missing dependencies?)\033[0m" ; fi
	@echo -n "  - build/bin/pangolin_loader:     " ; if [ -f build/bin/pangolin_loader ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (Missing dependencies? did you try make pangolin)\033[0m" ; fi
	@echo ""
	@echo "Tools/Debugger available: "
	@echo -n "  - build/bin/pointcloud_aligner:  " ; if [ -f build/bin/pointcloud_aligner ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (Missing dependencies (i.e. pcl)?)\033[0m" ; fi
	@echo -n "  - build/bin/dataset-generator:   " ; if [ -f build/bin/dataset-generator ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (Missing dependencies?)\033[0m" ; fi
	@echo -n "  - build/bin/io-inspect-file:     " ; if [ -f build/bin/io-inspect-file ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (Missing dependencies?)\033[0m" ; fi
	@echo -n "  - build/bin/io-readply:          " ; if [ -f build/bin/io-readply ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (Missing dependencies?)\033[0m" ; fi
	@echo ""
	@echo "The list of current use-case libraries available is:"
	@echo ""                                                                      
	@for f in `ls build/lib/lib*-library.so 2> /dev/null || echo Nothing` ; do echo $$f ; done
	@echo ""
	@echo "As a next step we suggest you to run \"make usecases\" or \"make slambench APPS=all\"."
	@echo ""
	@echo "================================================================================================================="
	@echo ""

usecases: 
	@echo ""
	@echo "================================================================================================================="
	@echo -e "Current list of compatible SLAM systems (alphabetical order). If you are using one of those SLAM algorithms, \033[1;31mplease refer to their respective publications\033[0m:"
	@echo ""

	@echo -n "  - ElasticFusion [Whelan et al, IJRR'16]: " ; if [ -f benchmarks/efusion ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (make efusion)\033[0m" ; fi
	@echo    "    repository: https://github.com/mp3guy/ElasticFusion"
	@echo    "    available targets are : efusion"
	@echo    ""

	@echo -n "  - InfiniTAMv2 [Kahler et al, ISMAR'15]: " ; if [ -f benchmarks/infinitam ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (make infinitam)\033[0m" ; fi
	@echo    "    repository: https://github.com/ethz-asl/infinitam"
	@echo    "    available targets are : infinitam"
	@echo ""

	@echo -n "  - KFusion [Newcombe et al. ISMAR'11]: " ; if [ -f benchmarks/kfusion ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (make kfusion)\033[0m" ; fi
	@echo    "    repository: https://github.com/GerhardR/kfusion"
	@echo    "    available targets are : kfusion"
	@echo    ""

	@echo -n "  - LSDSLAM [Engel et al, ECCV'14]: " ; if [ -f benchmarks/lsdslam ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (make lsdslam)\033[0m" ; fi
	@echo    "    repository: https://github.com/tum-vision/lsd_slam"
	@echo    "    available targets are : lsdslam"
	@echo ""

	@echo -n "  - MonoSLAM [Davison et al, TPAMI'07]: " ; if [ -f benchmarks/monoslam ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (make monoslam)\033[0m" ; fi
	@echo    "    repository: https://github.com/hanmekim/SceneLib2"
	@echo    "    available targets are : monoslam"
	@echo ""

	@echo -n "  - OKVIS [Leutenegger et al, IJRR'15]: " ; if [ -f benchmarks/okvis ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (make okvis)\033[0m" ; fi
	@echo    "    repository: https://github.com/ethz-asl/okvis"
	@echo    "    available targets are : okvis"
	@echo ""

	@echo -n "  - ORBSLAM2 [Mur-Artal et al, TOR'15 and TOR'17]: " ; if [ -f benchmarks/orbslam2 ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (make orbslam2)\033[0m" ; fi
	@echo    "    repository: https://github.com/raulmur/ORB_SLAM2"
	@echo    "    available targets are : orbslam2"
	@echo ""

	@echo -n "  - PTAM [Klein et al, ISMAR'07 and ECCV'08]: " ; if [ -f benchmarks/ptam ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (make ptam)\033[0m" ; fi
	@echo    "    repository: https://github.com/Oxford-PTAM/PTAM-GPL"
	@echo    "    available targets are : ptam"
	@echo ""

	@echo -n "  - SVO [Forster et al, ICRA'14]: " ; if [ -f benchmarks/svo ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (make svo)\033[0m" ; fi
	@echo    "    repository: https://github.com/uzh-rpg/rpg_svo"
	@echo    "    available targets are : svo"
	@echo ""

	@echo -n "  - FLAME [Greene et al, ICCV '17]: "; if [ -f benchmarks/flame ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (make flame)\033[0m" ; fi
	@echo    "    repository: https://github.com/mihaibujanca/flame"
	@echo    "    available targets are : flame"
	@echo ""

	@echo -n "  - ReFusion [Palazollo et al, IROS '19]: "; if [ -f benchmarks/flame ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (make refusion)\033[0m" ; fi
	@echo    "    repository: https://github.com/mihaibujanca/refusion"
	@echo    "    available targets are : refusion"
	@echo ""

	@echo -n "  - KinectFusion [Newcombe et al. ISMAR'11] (Christian Diller's implementation): " ; if [ -d benchmarks/kinectfusion/src/original ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (make kfusion)\033[0m" ; fi
	@echo    "    repository: https://github.com/mihaibujanca/KinectFusion"
	@echo    "    available targets are : kinectfusion"
	@echo    ""

	@echo "If you want to test SLAMBench with existing SLAM algorithms, once you have download it please run \"make slambench APPS=slam1,slam2,...\""
	@echo "   e.g. make slambench APPS=kfusion,orbslam2"
	@echo "   You can also use \"make slambench APPS=all\" to compile them all."
	@echo ""
	@echo "As a next step we suggest you run make datasets."
	@echo ""
	@echo "================================================================================================================="



efusion: 
	@echo "================================================================================================================="
	@echo    "  - ElasticFusion [Whelan et al, IJRR'16]: " 
	@echo    "    Original repository: https://github.com/mp3guy/ElasticFusion"
	@echo    "    Used repository: https://github.com/bbodin/ElasticFusion"
	@echo "================================================================================================================="
	@echo ""
	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir benchmarks/$@/src/ -p
	rm benchmarks/$@/src/original -rf
	git clone https://github.com/bbodin/ElasticFusion benchmarks/$@/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt


infinitam:
	@echo "================================================================================================================="
	@echo    "  - InfiniTAMv2 [Kahler et al, ISMAR'15]: " 
	@echo    "    Original repository: https://github.com/ethz-asl/infinitam"
	@echo    "    Used repository: https://github.com/bbodin/InfiniTAM"
	@echo "================================================================================================================="
	@echo ""
	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir benchmarks/infinitam/src/ -p
	rm benchmarks/infinitam/src/original -rf
	git clone https://github.com/bbodin/InfiniTAM.git benchmarks/infinitam/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/infinitam/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt


lsdslam:
	@echo "================================================================================================================="
	@echo    "  - LSDSLAM [Engel et al, ECCV'14]: " 
	@echo    "    Original repository: https://github.com/tum-vision/lsd_slam"
	@echo    "    Used repository : https://github.com/pamela-project/lsd_slam.git"  
	@echo "================================================================================================================="
	@echo ""
	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir benchmarks/lsdslam/src/ -p
	rm benchmarks/lsdslam/src/original -rf
	rm benchmarks/lsdslam/src/cpp -rf
	git clone --branch master https://github.com/pamela-project/lsd_slam.git benchmarks/lsdslam/src/original
	git clone --branch cpp    https://github.com/pamela-project/lsd_slam.git benchmarks/lsdslam/src/cpp
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/lsdslam/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

orbslam2:
	@echo "================================================================================================================="
	@echo    "  - ORBSLAM2 [Mur-Artal et al, TOR'15 and TOR'17]: "
	@echo    "    Original repository: https://github.com/raulmur/ORB_SLAM2"
	@echo    "    Used repository: https://github.com/pamela-project/ORB_SLAM2"  
	@echo "================================================================================================================="
	@echo ""
	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir benchmarks/orbslam2/src/ -p
	rm benchmarks/orbslam2/src/original -rf
	git clone --branch master  https://github.com/pamela-project/ORB_SLAM2.git benchmarks/orbslam2/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/orbslam2/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt


monoslam:
	@echo "================================================================================================================="
	@echo    "  - MonoSLAM [Davison et al, TPAMI'07]: " 
	@echo    "    Original repository: https://github.com/hanmekim/SceneLib2"
	@echo    "    Used repository: https://github.com/bbodin/SceneLib2"  
	@echo "================================================================================================================="
	@echo ""
	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir benchmarks/monoslam/src/ -p
	rm benchmarks/monoslam/src/original -rf
	git clone  https://github.com/bbodin/SceneLib2  benchmarks/monoslam/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/monoslam/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

ptam:
	@echo "================================================================================================================="
	@echo    "  - PTAM [Klein et al, ISMAR'07 and ECCV'08]: " 
	@echo    "    Original repository: https://github.com/Oxford-PTAM/PTAM-GPL"
	@echo    "    Used repository: https://github.com/bbodin/PTAM-GPL"  
	@echo "================================================================================================================="
	@echo ""
	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir benchmarks/ptam/src/ -p
	rm benchmarks/ptam/src/original -rf
	git clone   https://github.com/bbodin/PTAM-GPL  benchmarks/ptam/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/ptam/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

okvis:
	@echo "================================================================================================================="
	@echo    "  - OKVIS [Leutenegger et al, IJRR'15]: " 
	@echo    "    Original repository: https://github.com/ethz-asl/okvis"
	@echo    "    Used repository: https://github.com/bbodin/okvis"  
	@echo "================================================================================================================="
	@echo ""
	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir benchmarks/okvis/src/ -p
	rm benchmarks/okvis/src/original -rf
	git clone  https://github.com/bbodin/okvis   benchmarks/okvis/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/okvis/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

svo: 
	@echo "================================================================================================================="
	@echo    "  - SVO [Forster et al, ICRA'14]: " 
	@echo    "    Original repository: https://github.com/uzh-rpg/rpg_svo"
	@echo    "    Used repository: https://github.com/pamela-project/rpg_svo.git"  
	@echo "================================================================================================================="
	@echo ""
	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir benchmarks/svo/src/ -p
	rm benchmarks/svo/src/original -rf
	git clone  https://github.com/pamela-project/rpg_svo.git   benchmarks/svo/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/svo/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

kfusion:
	@echo "================================================================================================================="
	@echo    "  - KFusion [Newcombe et al. ISMAR'11] (original SLAMBench paper version)  "
	@echo    "    repository: https://github.com/GerhardR/kfusion"
	@echo    "    Used repository: https://github.com/pamela-project/kfusion"  
	@echo "================================================================================================================="
	@echo ""
	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir benchmarks/kfusion/src/ -p
	rm benchmarks/kfusion/src/original -rf
	git clone   https://github.com/pamela-project/kfusion   benchmarks/kfusion/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/kfusion/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

flame:
	@echo "================================================================================================================="
	@echo    "  - FLAME [Greene et al. ICCV'17]: "
	@echo    "    repository: https://github.com/robustrobotics/flame"
	@echo    "    Used repository: https://github.com/mihaibujanca/flame"
	@echo "================================================================================================================="
	@echo ""
	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir benchmarks/flame/src/original -p
	rm benchmarks/flame/src/original -rf
	git clone   https://github.com/mihaibujanca/flame   benchmarks/flame/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"      > benchmarks/flame/src/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

refusion:
	@echo "================================================================================================================="
	@echo    "  - ReFusion [Palazollo et al. IROS'19]: "
	@echo    "    repository: https://github.com/PRBonn/refusion"
	@echo    "    Used repository: https://github.com/mihaibujanca/refusion"
	@echo "================================================================================================================="
	@echo ""
	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir benchmarks/refusion/src/original -p
	rm benchmarks/refusion/src/original -rf
	git clone   https://github.com/mihaibujanca/refusion   benchmarks/refusion/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"      > benchmarks/refusion/src/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

open_vins:
	@echo "================================================================================================================="
	@echo    "  - OpenVINS [Geneva et al. IROS'19]: "
	@echo    "    repository: https://github.com/rpng/open_vins"
	@echo    "    Used repository: https://github.com/mihaibujanca/open_vins"
	@echo "================================================================================================================="
	@echo ""
	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir benchmarks/open_vins/src/original -p
	rm benchmarks/open_vins/src/original -rf
	git clone   https://github.com/mihaibujanca/open_vins   benchmarks/open_vins/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"      > benchmarks/open_vins/src/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

kinectfusion:
	@echo "================================================================================================================="
	@echo    "  - KinectFusion [Newcombe et al. ISMAR'11] (Christian Diller's implementation)"
	@echo    "    Original repository: https://github.com/chrdiller/KinectFusionApp"
	@echo    "    Used repository: https://github.com/mihaibujanca/KinectFusion"
	@echo "================================================================================================================="
	@echo ""
	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/kinectfusion/src/original
	rm benchmarks/kinectfusion/src/original -rf
	git clone --recursive https://github.com/mihaibujanca/KinectFusion benchmarks/kinectfusion/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

.PHONY: efusion infinitam kfusion lsdslam monoslam okvis orbslam2 ptam svo flame open_vins refusion kinectfusion
algorithms : efusion infinitam kfusion lsdslam monoslam okvis orbslam2 ptam svo flame open_vins refusion kinectfusion


datasets :
	@echo ""
	@echo "================================================================================================================="
	@echo "SLAMBench integrates tools to automatically generate files compatible with SLAMBench from existing datasets."
	@echo ""
	@echo "The list of current dataset is:                                                                                 "
	@for f in `find datasets/ | grep [.]slam` ; do echo "   - $$f" ; done
	@echo ""
	@echo "If you do not find datasets in this list, you can use make to download them (make datasetslist). "
	@echo "Here is a list of the datasets available."
	@echo -e "If you are using one of those dataset, \033[1;31mplease refer to their respective publications\033[0m:"
	@echo "   - TUM RGB-D SLAM dataset [Sturm et al, IROS'12]: https://vision.in.tum.de/data/datasets/rgbd-dataset"
	@echo "   - ICL-NUIM dataset [Handa et al, ICRA'14]: https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html"
	@echo "   - EuRoC MAV Dataset [Burri et al, IJJR'16]: https://projects.asl.ethz.ch/datasets/doku.php"
	@echo "   - SVO sample dataset [Forster et al, ICRA 2014]: https://github.com/uzh-rpg/rpg_svo"
	@echo "   - Bonn RGB-D Dynamic Dataset [Palazzolo et al, IROS'19]: http://www.ipb.uni-bonn.de/data/rgbd-dynamic-dataset/"
	@echo "   - UZH-FPV Drone Racing Dataset [Delmerico et al, ICRA'19]: http://rpg.ifi.uzh.ch/uzh-fpv.html"
	@echo "================================================================================================================="

datasetslist:
	@echo ""
	@echo "### ICL-NUIM Living Room"
	@echo ""
	@echo "make ./datasets/ICL_NUIM/living_room_traj0_loop.slam"
	@echo "make ./datasets/ICL_NUIM/living_room_traj1_loop.slam"
	@echo "make ./datasets/ICL_NUIM/living_room_traj2_loop.slam"
	@echo "make ./datasets/ICL_NUIM/living_room_traj3_loop.slam"
	@echo ""
	@echo ""
	@echo "### TUM Testing and Debugging"
	@echo ""
	@echo "make ./datasets/TUM/freiburg1/rgbd_dataset_freiburg1_xyz.slam"
	@echo "make ./datasets/TUM/freiburg1/rgbd_dataset_freiburg1_rpy.slam"
	@echo "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_xyz.slam"
	@echo "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_rpy.slam"
	@echo ""
	@echo ""
	@echo "### TUM Handheld SLAM"
	@echo ""
	@echo "make ./datasets/TUM/freiburg1/rgbd_dataset_freiburg1_360.slam"
	@echo "make ./datasets/TUM/freiburg1/rgbd_dataset_freiburg1_floor.slam"
	@echo "make ./datasets/TUM/freiburg1/rgbd_dataset_freiburg1_desk.slam"
	@echo "make ./datasets/TUM/freiburg1/rgbd_dataset_freiburg1_desk2.slam"
	@echo "make ./datasets/TUM/freiburg1/rgbd_dataset_freiburg1_room.slam"
	@echo ""
	@echo "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_360_hemisphere.slam"
	@echo "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_360_kidnap.slam"
	@echo ""
	@echo "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_desk.slam"
	@echo "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_desk_with_person.slam"
	@echo "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_large_no_loop.slam"
	@echo "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_large_with_loop.slam"
	@echo ""
	@echo "### TUM Robot SLAM"
	@echo ""
	@echo "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_pioneer_360.slam"
	@echo "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_pioneer_slam.slam"
	@echo "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_pioneer_slam2.slam"
	@echo "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_pioneer_slam3.slam"
	@echo ""
	@echo "##### TUM Extra dataset"
	@echo ""
	@echo "## https://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household.tgz"
	@echo ""
	@echo "### ETH Illumination TUM-based"
	@echo ""
	@echo "make ./datasets/ETHI/ethl_real_flash.slam"
	@echo "make ./datasets/ETHI/ethl_real_local.slam"
	@echo "make ./datasets/ETHI/ethl_real_global.slam"
	@echo ""
	@echo "### ETH Illumination ICLNUIM-based"
	@echo ""
	@echo "make ./datasets/ETHI/ethl_syn1.slam"
	@echo "make ./datasets/ETHI/ethl_syn1_local.slam"
	@echo "make ./datasets/ETHI/ethl_syn1_global.slam"
	@echo "make ./datasets/ETHI/ethl_syn1_loc_glo.slam"
	@echo "make ./datasets/ETHI/ethl_syn1_flash.slam"
	@echo "make ./datasets/ETHI/ethl_syn2.slam"
	@echo "make ./datasets/ETHI/ethl_syn2_local.slam"
	@echo "make ./datasets/ETHI/ethl_syn2_global.slam"
	@echo "make ./datasets/ETHI/ethl_syn2_loc_glo.slam"
	@echo "make ./datasets/ETHI/ethl_syn2_flash.slam"
	@echo ""
	@echo ""
	@echo ""
	@echo "### EuRoC MAV Machine Hall"
	@echo ""
	@echo "make ./datasets/EuRoCMAV/machine_hall/MH_01_easy/MH_01_easy.slam"
	@echo "make ./datasets/EuRoCMAV/machine_hall/MH_02_easy/MH_02_easy.slam"
	@echo "make ./datasets/EuRoCMAV/machine_hall/MH_03_medium/MH_03_medium.slam"
	@echo "make ./datasets/EuRoCMAV/machine_hall/MH_04_difficult/MH_04_difficult.slam"
	@echo "make ./datasets/EuRoCMAV/machine_hall/MH_05_difficult/MH_05_difficult.slam"
	@echo ""
	@echo "### EuRoC MAV Vicon Room"
	@echo ""
	@echo "make ./datasets/EuRoCMAV/vicon_room1/V1_01_easy/V1_01_easy.slam"
	@echo "make ./datasets/EuRoCMAV/vicon_room1/V1_02_medium/V1_02_medium.slam"
	@echo "make ./datasets/EuRoCMAV/vicon_room1/V1_03_difficult/V1_03_difficult.slam"
	@echo "make ./datasets/EuRoCMAV/vicon_room2/V2_01_easy/V2_01_easy.slam"
	@echo "make ./datasets/EuRoCMAV/vicon_room2/V2_02_medium/V2_02_medium.slam"
	@echo "make ./datasets/EuRoCMAV/vicon_room2/V2_03_difficult/V2_03_difficult.slam"
	@echo ""
	@echo ""
	@echo "### Bonn Balloon"
	@echo ""
	@echo "make ./datasets/BONN/rgbd_bonn_balloon.slam"
	@echo "make ./datasets/BONN/rgbd_bonn_balloon2.slam"
	@echo "make ./datasets/BONN/rgbd_bonn_balloon_tracking.slam"
	@echo "make ./datasets/BONN/rgbd_bonn_balloon_tracking2.slam"
	@echo ""
	@echo ""
	@echo "### Bonn People"
	@echo ""
	@echo "make ./datasets/BONN/rgbd_bonn_crowd.slam"
	@echo "make ./datasets/BONN/rgbd_bonn_crowd2.slam"
	@echo "make ./datasets/BONN/rgbd_bonn_crowd3.slam"
	@echo "make ./datasets/BONN/rgbd_bonn_person_tracking.slam"
	@echo "make ./datasets/BONN/rgbd_bonn_person_tracking2.slam"
	@echo ""
	@echo ""
	@echo "### Bonn Boxes"
	@echo ""
	@echo "make ./datasets/BONN/rgbd_bonn_kidnapping_box.slam"
	@echo "make ./datasets/BONN/rgbd_bonn_kidnapping_box2.slam"
	@echo "make ./datasets/BONN/rgbd_bonn_moving_nonobstructing_box.slam"
	@echo "make ./datasets/BONN/rgbd_bonn_moving_nonobstructing_box2.slam"
	@echo "make ./datasets/BONN/rgbd_bonn_moving_obstructing_box.slam"
	@echo "make ./datasets/BONN/rgbd_bonn_moving_obstructing_box2.slam"
	@echo "make ./datasets/BONN/rgbd_bonn_placing_nonobstructing_box.slam"
	@echo "make ./datasets/BONN/rgbd_bonn_placing_nonobstructing_box2.slam"
	@echo "make ./datasets/BONN/rgbd_bonn_placing_nonobstructing_box3.slam"
	@echo "make ./datasets/BONN/rgbd_bonn_placing_obstructing_box.slam"
	@echo "make ./datasets/BONN/rgbd_bonn_removing_nonobstructing_box.slam"
	@echo "make ./datasets/BONN/rgbd_bonn_removing_nonobstructing_box2.slam"
	@echo "make ./datasets/BONN/rgbd_bonn_removing_obstructing_box.slam"
	@echo ""
	@echo ""
	@echo "### Bonn Synchronous and Static"
	@echo ""
	@echo "make ./datasets/BONN/rgbd_bonn_synchronous.slam"
	@echo "make ./datasets/BONN/rgbd_bonn_synchronous2.slam"
	@echo "make ./datasets/BONN/rgbd_bonn_static.slam"
	@echo "make ./datasets/BONN/rgbd_bonn_static_close_far.slam"
	@echo ""
	@echo ""
	@echo "### UZH FPV Drone Indoor forward facing"
	@echo ""
	@echo "make ./datasets/UZHFPV/indoor_forward_3_snapdragon_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_forward_3_davis_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_forward_5_snapdragon_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_forward_5_davis_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_forward_6_snapdragon_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_forward_6_davis_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_forward_7_snapdragon_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_forward_7_davis_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_forward_8_snapdragon.slam"
	@echo "make ./datasets/UZHFPV/indoor_forward_8_davis.slam"
	@echo "make ./datasets/UZHFPV/indoor_forward_9_snapdragon_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_forward_9_davis_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_forward_10_snapdragon_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_forward_10_davis_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_forward_11_snapdragon.slam"
	@echo "make ./datasets/UZHFPV/indoor_forward_11_davis.slam"
	@echo "make ./datasets/UZHFPV/indoor_forward_12_snapdragon.slam"
	@echo "make ./datasets/UZHFPV/indoor_forward_12_davis.slam"
	@echo ""
	@echo ""
	@echo "### UZH FPV Drone Indoor 45 degree downward facing"
	@echo ""
	@echo "make ./datasets/UZHFPV/indoor_45_1_snapdragon.slam"
	@echo "make ./datasets/UZHFPV/indoor_45_1_davis.slam"
	@echo "make ./datasets/UZHFPV/indoor_45_2_snapdragon_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_45_2_davis_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_45_3_snapdragon.slam"
	@echo "make ./datasets/UZHFPV/indoor_45_3_davis.slam"
	@echo "make ./datasets/UZHFPV/indoor_45_4_snapdragon_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_45_4_davis_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_45_9_snapdragon_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_45_9_davis_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_45_11_snapdragon.slam"
	@echo "make ./datasets/UZHFPV/indoor_45_11_davis.slam"
	@echo "make ./datasets/UZHFPV/indoor_45_12_snapdragon_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_45_12_davis_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_45_13_snapdragon_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_45_13_davis_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_45_14_snapdragon_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_45_14_davis_with_gt.slam"
	@echo "make ./datasets/UZHFPV/indoor_45_16_snapdragon.slam"
	@echo "make ./datasets/UZHFPV/indoor_45_16_davis.slam"
	@echo ""
	@echo ""
	@echo "### UZH FPV Drone Outdoor forward facing"
	@echo ""
	@echo "make ./datasets/UZHFPV/outdoor_forward_1_snapdragon_with_gt.slam"
	@echo "make ./datasets/UZHFPV/outdoor_forward_1_davis_with_gt.slam"
	@echo "make ./datasets/UZHFPV/outdoor_forward_2_snapdragon.slam"
	@echo "make ./datasets/UZHFPV/outdoor_forward_2_davis.slam"
	@echo "make ./datasets/UZHFPV/outdoor_forward_3_snapdragon_with_gt.slam"
	@echo "make ./datasets/UZHFPV/outdoor_forward_3_davis_with_gt.slam"
	@echo "make ./datasets/UZHFPV/outdoor_forward_5_snapdragon_with_gt.slam"
	@echo "make ./datasets/UZHFPV/outdoor_forward_5_davis_with_gt.slam"
	@echo "make ./datasets/UZHFPV/outdoor_forward_6_snapdragon.slam"
	@echo "make ./datasets/UZHFPV/outdoor_forward_6_davis.slam"
	@echo "make ./datasets/UZHFPV/outdoor_forward_9_snapdragon.slam"
	@echo "make ./datasets/UZHFPV/outdoor_forward_9_davis.slam"
	@echo "make ./datasets/UZHFPV/outdoor_forward_10_snapdragon.slam"
	@echo "make ./datasets/UZHFPV/outdoor_forward_10_davis.slam"
	@echo ""
	@echo ""
	@echo "### UZH FPV Drone Outdoor 45 degree downward facing"
	@echo ""
	@echo "make ./datasets/UZHFPV/outdoor_45_1_snapdragon_with_gt.slam"
	@echo "make ./datasets/UZHFPV/outdoor_45_1_davis_with_gt.slam"
	@echo "make ./datasets/UZHFPV/outdoor_45_2_snapdragon.slam"
	@echo "make ./datasets/UZHFPV/outdoor_45_2_davis.slam"
	@echo ""
	@echo ""
	@echo "### SVO test dataset"
	@echo ""
	@echo "make datasets/SVO/artificial.slam"
	@echo ""
	@echo ""
	@echo "### OpenLORIS-Scene datasets"
	@echo ""
	@echo "# SLAMBench cannot download OpenLORIS data for you. Please manually download the package data (*-package.tar), put them into ./datasets/OpenLORIS/, and then run the following commands to build data sequences in each scene."
	@echo "# Please make sure 7z has been installed (try with 'sudo apt-get install p7zip' if you are using Ubuntu)."
	@echo "make ./datasets/OpenLORIS/office1.all"
	@echo "make ./datasets/OpenLORIS/corridor1.all"
	@echo "make ./datasets/OpenLORIS/home1.all"
	@echo "make ./datasets/OpenLORIS/cafe1.all"
	@echo "make ./datasets/OpenLORIS/market1.all"
	@echo ""
	@echo "# You can also build only one sequence, for example:"
	@echo "make ./datasets/OpenLORIS/office1/office1-1.slam"
	@echo ""
	@echo ""
	@echo "================================================================================================================="
	@echo -e "If you are using one of those dataset, \033[1;31mplease refer to their respective publications\033[0m:"
	@echo "   - TUM RGB-D SLAM dataset [Sturm et al, IROS'12]: https://vision.in.tum.de/data/datasets/rgbd-dataset"
	@echo "   - ICL-NUIM dataset [Handa et al, ICRA'14]: https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html"
	@echo "   - SVO sample dataset [Forster et al, ICRA '14]: https://github.com/uzh-rpg/rpg_svo"
	@echo "   - EuRoC MAV Dataset [Burri et al, IJJR'16]: https://projects.asl.ethz.ch/datasets/doku.php"
	@echo "   - ETHI Dataset [Park et al, ICRA'17]: https://cvg.ethz.ch/research/illumination-change-robust-dslam/"
	@echo "   - Bonn RGB-D Dynamic Dataset [Palazzolo et al, IROS'19]: http://www.ipb.uni-bonn.de/data/rgbd-dynamic-dataset/"
	@echo "   - UZH-FPV Drone Racing Dataset [Delmerico et al, ICRA'19]: http://rpg.ifi.uzh.ch/uzh-fpv.html"
	@echo "   - OpenLORIS-Scene datasets [Shi et al, ICRA'20]: https://lifelong-robotic-vision.github.io/dataset/scene"
	@echo "================================================================================================================="

.PHONY: slambench benchmarks benchmarkslist datasets datasetslist


####################################
#### DATA SET GENERATION        ####
####################################


#### OpenLORIS-Scene
####################

datasets/OpenLORIS/%.7z :  # Example : $* = office1/office1-3
	# extract 7z from the tar file of the scene, e.g. office1-1_7-package.tar
	for f in $(@D)*-package.tar; do echo $$f && mkdir -p $(@D) && tar xvf $$f -C $(@D); done
	if [ ! -f $@ ]; then echo "Could not find $(@D)*-package.tar or $@. Please download the data first."; fi

datasets/OpenLORIS/%.dir : ./datasets/OpenLORIS/%.7z
	7z x $< -o$(@D) -aos
	# add the '.dir' suffix
	mv $(subst .7z,,$<) $(subst .7z,.dir,$<)

datasets/OpenLORIS/%.slam : ./datasets/OpenLORIS/%.dir
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d OpenLORIS -i $</ -o $@
	echo "Generated $@"

datasets/OpenLORIS/%.all :
	# if there are any tar, untar them; then build each 7z into a slam file
	scene=datasets/OpenLORIS/$*; \
	if [ -f $$scene*-package.tar ]; then \
		mkdir -p $$scene; \
		for f in $$scene*-package.tar; do \
			echo $$f && tar xvf $$f -C $$scene; \
		done; \
	fi; \
	for f in $$scene/*.7z; do \
		target=`echo $$f | tr .7z .sl`am ; \
		echo =============== $$target =============== ; \
		$(MAKE) $$target; \
	done

.SECONDARY: $(OBJS)


#### EuRoCMAV
###############

./datasets/EuRoCMAV/%.zip :  # Example : $* = machine_hall/MH_01_easy/MH_01_easy
	mkdir -p $(@D)
	cd $(@D)  &&  ${WGET} "http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/$*.zip"


./datasets/EuRoCMAV/%.dir : ./datasets/EuRoCMAV/%.zip
	mkdir $@
	unzip $< -d $@

./datasets/EuRoCMAV/%.slam :  ./datasets/EuRoCMAV/%.dir 
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d eurocmav -i $</mav0 -o $@ -imu true -stereo true -gt true

#### TUM      
###############

./datasets/TUM/%.tgz :  # Example : $* = freiburg2/rgbd_dataset_freiburg2_desk 
	mkdir -p $(@D)
	cd $(@D)  &&  ${WGET} "http://vision.in.tum.de/rgbd/dataset/$*.tgz"

./datasets/TUM/%.dir : ./datasets/TUM/%.tgz
	mkdir $@
	tar xzf $< -C $@

./datasets/TUM/%.slam :  ./datasets/TUM/%.dir 
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d tum -i $</* -o $@ -grey true -rgb true -gt true -depth true -accelerometer true 

#### ICL-NUIM      
###############
datasets/ICL_NUIM/living-room.ply :  datasets/ICL_NUIM/living-room.ply.tar.gz
	cd datasets/ICL_NUIM  && tar xzf living-room.ply.tar.gz
	touch datasets/ICL_NUIM/living-room.ply # This is a fix to ensure not regenerating the file again because of file create date

datasets/ICL_NUIM/living-room.ply.tar.gz : 
	mkdir -p  datasets/ICL_NUIM
	cd datasets/ICL_NUIM  && ${WGET} "http://www.doc.ic.ac.uk/~ahanda/living-room.ply.tar.gz"

datasets/ICL_NUIM/%_loop.tgz : 
	mkdir -p  datasets/ICL_NUIM
	cd datasets/ICL_NUIM  && ${WGET} "http://www.doc.ic.ac.uk/~ahanda/$*_loop.tgz"

datasets/ICL_NUIM/%_loop.dir :  datasets/ICL_NUIM/%_loop.tgz
	mkdir -p $@
	tar xzf $< -C $@ 

datasets/ICL_NUIM/living_room_traj%_loop.slam : datasets/ICL_NUIM/living_room_traj%_loop.dir datasets/ICL_NUIM/living-room.ply
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d iclnuim -i $< -o $@ -ply  datasets/ICL_NUIM/living-room.ply -grey true -rgb true -gt true -depth true -pf true 

datasets/ICL_NUIM/living_room_traj%_loop_neg.slam : datasets/ICL_NUIM/living_room_traj%_loop.dir datasets/ICL_NUIM/living-room.ply
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d iclnuim -i $< -o $@ -ply  datasets/ICL_NUIM/living-room.ply -grey true -rgb true -gt true -depth true -pf false 


datasets/ICL_NUIM/%.gt.freiburg : 
	${ECHO}  "Download ground truth trajectory..."
	mkdir -p datasets/ICL_NUIM/
	cd datasets/ICL_NUIM/  &&  if test -x livingRoom$*.gt.freiburg ; then ${ECHO} "Done" ; else ${WGET} "http://www.doc.ic.ac.uk/~ahanda/VaFRIC/$*.gt.freiburg" ; fi

datasets/ICL_KLG/dyson_lab.klg :
	mkdir -p datasets/ICL_KLG/
	cd datasets/ICL_KLG/  &&  ${WGET} http://www.doc.ic.ac.uk/%7Esleutene/datasets/elasticfusion/dyson_lab.klg

#### SVO-artificial
###############
datasets/SVO/artificial.tar.gz:
	mkdir -p datasets/SVO/
	cd datasets/SVO && ${WGET} -O artificial.tar.gz "http://rpg.ifi.uzh.ch/datasets/sin2_tex2_h1_v8_d.tar.gz"

datasets/SVO/artificial.dir: ./datasets/SVO/artificial.tar.gz
	mkdir -p $@
	tar -xzf $< -C $@

datasets/SVO/artificial.slam: ./datasets/SVO/artificial.dir
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d svo -i $</sin2_tex2_h1_v8_d -o $@  


#### BONN
#################

./datasets/BONN/%.ply :  ./datasets/BONN/%.zip
	unzip $< -d datasets/BONN
	touch $@ # This is a fix to ensure not regenerating the file again because of file create date

./datasets/BONN/%.zip :  # Example : $* = rgbd_bonn_balloon
	mkdir -p $(@D)
	cd $(@D)  &&  ${WGET} "http://www.ipb.uni-bonn.de/html/projects/rgbd_dynamic2019/$*.zip"

./datasets/BONN/%.dir : ./datasets/BONN/%.zip
	mkdir $@
	unzip $< -d $@

./datasets/BONN/%.slam : ./datasets/BONN/%.dir ./datasets/BONN/rgbd_bonn_groundtruth_1mm_section.ply
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d bonn -i $</* -o $@ -grey true -rgb true -gt true -depth true -ply datasets/BONN/rgbd_bonn_groundtruth_1mm_section.ply

#./datasets/BONN/%.slam : ./datasets/BONN/%.dir ./datasets/BONN/rgbd_bonn_groundtruth.ply
#	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
#	./build/bin/dataset-generator -d bonn -i $</* -o $@ -grey true -rgb true -gt true -depth true -ply datasets/BONN/rgbd_bonn_groundtruth.ply


#### UZHFPV Drone
###############

./datasets/UZHFPV/%.zip :  # Example : $* = indoor_foward_3_snapdragon_with_gt
	echo download $*.zip
	mkdir -p $(@D)
	cd $(@D)  &&  ${WGET} "http://rpg.ifi.uzh.ch/datasets/uzh-fpv/$*.zip"

./datasets/UZHFPV/%.dir : ./datasets/UZHFPV/%.zip
	mkdir $@
	unzip $< -d $@

./datasets/UZHFPV/%_snapdragon_with_gt.slam :  ./datasets/UZHFPV/%_snapdragon_with_gt.dir
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d uzhfpv -i $< -o $@ -imu true --stereo true --event false -gt true

./datasets/UZHFPV/%_davis_with_gt.slam :  ./datasets/UZHFPV/%_davis_with_gt.dir
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d uzhfpv -i $< -o $@ -imu true --stereo false --event true -gt true

./datasets/UZHFPV/%_snapdragon.slam :  ./datasets/UZHFPV/%_snapdragon.dir
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d uzhfpv -i $< -o $@ -imu true --stereo true --event false -gt false

./datasets/UZHFPV/%_davis.slam :  ./datasets/UZHFPV/%_davis.dir
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d uzhfpv -i $< -o $@ -imu true --stereo false --event true -gt false


#### ETH Illumination
###############
./datasets/ETHI/%.zip :
	echo download $*.zip
	mkdir -p $(@D)
	cd $(@D)  &&  ${WGET} "https://cvg.ethz.ch/research/illumination-change-robust-dslam/$*.zip"

./datasets/ETHI/%.dir : ./datasets/ETHI/%.zip
	mkdir $@
	unzip $< -d $@

### TUM-based sequences contain "real", ICLNUIM-based sequences contain "syn"
### Add accelerometer.txt to prevent TUM breaking. Make sure depth.txt and rgb.txt exist in their respective folders.
###
./datasets/ETHI/%.slam : ./datasets/ETHI/%.dir ./datasets/ICL_NUIM/living-room.ply.tar.gz
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	for d in $</*; do \
  		echo "$$d"; \
		case "$(@F)" in \
			(*real*) touch "$$d"/accelerometer.txt; \
			 		 cp "$$d"/depth.txt "$$d"/depth/; \
			 		 cp "$$d"/rgb.txt "$$d"/rgb/; \
			 		 ./build/bin/dataset-generator -d tum -i "$$d" -o $@ -grey true -rgb true -gt true -depth true -accelerometer false ;; \
			(*syn*) cd "$$d"; \
					for file in depth/*.png; do \
						  base=`basename -- "$$file" .png`;\
						  mv "$$file" "scene_00_$$base.depth.png"; \
					done;\
					for file in rgb/*.png; do \
						  base=`basename -- "$$file"`;\
						  mv "$$file" "scene_00_$$base"; \
					done;\
					cd -; \
					./build/bin/dataset-generator -d iclnuim -i "$$d" -o $@ -ply  datasets/ICL_NUIM/living-room.ply -grey true -rgb true -gt true -depth true -pf false ;;\
		esac \
	done;

./datasets/ETHI/all :./datasets/ICL_NUIM/living-room.ply.tar.gz \
					./datasets/ETHI/ethl_real_flash.slam \
					./datasets/ETHI/ethl_real_local.slam \
					./datasets/ETHI/ethl_real_global.slam \
					./datasets/ETHI/ethl_syn1.slam \
					./datasets/ETHI/ethl_syn1_local.slam \
					./datasets/ETHI/ethl_syn1_global.slam \
					./datasets/ETHI/ethl_syn1_loc_glo.slam \
					./datasets/ETHI/ethl_syn1_flash.slam \
					./datasets/ETHI/ethl_syn2.slam \
					./datasets/ETHI/ethl_syn2_local.slam \
					./datasets/ETHI/ethl_syn2_global.slam \
					./datasets/ETHI/ethl_syn2_loc_glo.slam \
					./datasets/ETHI/ethl_syn2_flash.slam
#if echo $(@F) | grep "syn" ; then make ./datasets/ICL_NUIM/living-room.ply.tar.gz; \
#		./build/bin/dataset-generator -d iclnuim -i $</* -o $@ -ply  datasets/ICL_NUIM/living-room.ply -grey true -rgb true -gt true -depth true -pf true \

#### ORBSLAM Voc
###############

./benchmarks/orbslam2/src/original/Vocabulary/ORBvoc.txt : ./benchmarks/orbslam2/src/original/Vocabulary/ORBvoc.txt.tar.gz
	cd ./benchmarks/orbslam2/src/original/Vocabulary/ && tar -xf ORBvoc.txt.tar.gz

.PRECIOUS: \
./datasets/TUM/%.tgz \
./datasets/TUM/%.dir \
./datasets/TUM/%.raw \
./datasets/ICL_NUIM/living_room_traj%_loop.tgz \
./datasets/ICL_NUIM/living_room_traj%_loop.dir \
./datasets/ICL_NUIM/livingRoom%.gt.freiburg \
./datasets/ICL_NUIM/living_room_traj%_loop.raw \
./datasets/BONN/%.zip \
./datasets/BONN/%.ply \
./datasets/BONN/%.dir \
./datasets/UZHFPV/%.dir \
./datasets/UZHFPV/%.zip \
./datasets/ETHI/%.dir \
./datasets/ETHI/%.zip

####################################
####    BUILD/CLEAN TOOL        ####
####################################

#### CLEAN/CLEANALL TOOLS ####
clean :
	rm -rf build install android-build android-install docker/tmp/

cleandatasets :
	find datasets/ | grep [.]slam | xargs rm 2> /dev/null || true
	@echo "Datasets Cleaned"

cleandeps :
	rm -rf deps

cleanall : clean cleandatasets cleandeps

.PHONY: clean cleandeps cleandatasets cleanall


####################################
####        DEMO PART           ####
####################################

demo-prepare :
	make slambench APPS=all
	make datasets/ICL_NUIM/living_room_traj2_loop.slam
	make datasets/EuRoCMAV/machine_hall/MH_01_easy/MH_01_easy.slam
	make datasets/TUM/freiburg2/rgbd_dataset_freiburg2_desk.slam

demo-lib :
	clear
	@${ECHO} -e "                        "
	@${ECHO} -e " ======================="
	@${ECHO} -e " Pick UI                "
	@${ECHO} -e "   *  depends on the purpose     "
	@${ECHO} -e " ======================="
	@${ECHO} -e "                        "
	@for d in `find ./build/bin/ -name *_loader ` ; do ${ECHO} " $$d" ; done
	@${ECHO} -e "                        "
	@${ECHO} -e " ======================="
	@${ECHO} -e " Pick dataset           "
	@${ECHO} -e "   *  the slam format includes camera parameters "
	@${ECHO} -e " ======================="
	@${ECHO} -e "                        "
	@for d in `find datasets/ | grep [.]slam` ; do ${ECHO} " -i $$d" ; done
	@${ECHO} -e "                        "
	@${ECHO} -e " ======================="
	@${ECHO} -e " Pick libraries          "
	@${ECHO} -e "   *  may requires parameters"
	@${ECHO} -e "   *  do not need to be open source"
	@${ECHO} -e " ======================="
	@${ECHO} -e "                        "
	@for d in `find  build/lib/| grep [-]library[.]so` ; do ${ECHO} " -load $$d" ; done
	@${ECHO} -e "                        "
	@${ECHO} -e "                        "
	@${ECHO} -e " ======================="


####################################
####        TEST PART           ####
####################################

regression_test :
	make -C docker fastCI-docker-image # First test compilation of slambench and run kfusion
	make -C docker ubuntu-14.04-docker-image # Second compilation for embedded systems such as Odroid or TK1
# third run slambench for multiple datasets and check ATE and Memory usage

test :
	make -C docker fastCI-docker-image

