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
	${DEPS_ENV} cmake -U -Bbuild -H.  -DAPPS="${APPS}"  ${DEPS_ARGS} -D"CMAKE_MODULE_PATH:PATH=${ROOT_DIR}/cmake_modules"


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
	@echo    "  - KFusion [Newcombe et al. ISMAR'11]: " 
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


.PHONY: efusion infinitam kfusion lsdslam monoslam okvis orbslam2 ptam svo flame
algorithms : efusion infinitam kfusion lsdslam monoslam okvis orbslam2 ptam svo flame


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
	@echo "### SVO test dataset"
	@echo ""
	@echo "make datasets/SVO/artificial.slam"
	@echo ""
	@echo ""
	@echo "================================================================================================================="
	@echo -e "If you are using one of those dataset, \033[1;31mplease refer to their respective publications\033[0m:"
	@echo "   - TUM RGB-D SLAM dataset [Sturm et al, IROS'12]: https://vision.in.tum.de/data/datasets/rgbd-dataset"
	@echo "   - ICL-NUIM dataset [Handa et al, ICRA'14]: https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html"
	@echo "   - EuRoC MAV Dataset [Burri et al, IJJR'16]: https://projects.asl.ethz.ch/datasets/doku.php"
	@echo "   - SVO sample dataset [Forster et al, ICRA 2014]: https://github.com/uzh-rpg/rpg_svo"
	@echo "   - Bonn RGB-D Dynamic Dataset [Palazzolo et al, IROS'19]: http://www.ipb.uni-bonn.de/data/rgbd-dynamic-dataset/"
	@echo "================================================================================================================="

.PHONY: slambench benchmarks benchmarkslist datasets datasetslist


####################################
#### DATA SET GENERATION        ####
####################################



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

./datasets/BONN/%.zip :  # Example : $* = rgbd_bonn_balloon
	mkdir -p $(@D)
	cd $(@D)  &&  ${WGET} "http://www.ipb.uni-bonn.de/html/projects/rgbd_dynamic2019/$*.zip"

./datasets/BONN/%.dir : ./datasets/BONN/%.zip
	mkdir $@
	unzip $< -d $@

./datasets/BONN/%.slam :  ./datasets/BONN/%.dir
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d bonn -i $</* -o $@ -grey true -rgb true -gt true -depth true


#### ORBSLAM Voc
#################

./benchmarks/orbslam2/src/original/Vocabulary/ORBvoc.txt : ./benchmarks/orbslam2/src/original/Vocabulary/ORBvoc.txt.tar.gz
	cd ./benchmarks/orbslam2/src/original/Vocabulary/ && tar -xf ORBvoc.txt.tar.gz

.PRECIOUS: ./datasets/TUM/%.tgz ./datasets/TUM/%.dir ./datasets/TUM/%.raw datasets/ICL_NUIM/living_room_traj%_loop.tgz ./datasets/TUM/%.raw datasets/ICL_NUIM/living_room_traj%_loop.dir datasets/ICL_NUIM/livingRoom%.gt.freiburg datasets/ICL_NUIM/living_room_traj%_loop.raw

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

