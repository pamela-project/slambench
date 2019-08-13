usecases:
	@echo ""
	@echo "================================================================================================================="
	@echo -e "Current list of compatible SLAM systems (alphabetical order). If you are using any of the following SLAM algorithms, \033[1;31mplease refer to their respective publications\033[0m:"
	@echo ""
	@echo -n "  - ElasticFusion [Whelan et al, IJRR'16] : " ; if [ -d benchmarks/efusion/src/original ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (make efusion)\033[0m" ; fi
	@echo    "    repository: https://github.com/mp3guy/ElasticFusion"
	@echo    "    available targets are : efusion"
	@echo    ""

	@echo -n "  - InfiniTAMv2 [Kahler et al, ISMAR'15] : " ; if [ -d benchmarks/infinitam/src/original ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (make infinitam)\033[0m" ; fi
	@echo    "    repository: https://github.com/ethz-asl/infinitam"
	@echo    "    available targets are : infinitam"
	@echo    ""

	@echo -n "  - LSDSLAM [Engel et al, ECCV'14] : " ; if [ -d benchmarks/lsdslam/src/original ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (make lsdslam)\033[0m" ; fi
	@echo    "    repository: https://github.com/tum-vision/lsd_slam"
	@echo    "    available targets are : lsdslam"
	@echo    ""

	@echo -n "  - LSDSLAM [Engel et al, ECCV'14] : " ; if [ -d benchmarks/lsdslam/src/cpp ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (make lsdslam)\033[0m" ; fi
	@echo    "    repository: https://github.com/tum-vision/lsd_slam"
	@echo    "    available targets are : lsdslam"
	@echo    ""

	@echo -n "  - ORBSLAM2 [Mur-Artal et al, TOR'15 and TOR'17] : " ; if [ -d benchmarks/orbslam2/src/original ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (make orbslam2)\033[0m" ; fi
	@echo    "    repository: https://github.com/raulmur/ORB_SLAM2"
	@echo    "    available targets are : orbslam2"
	@echo    ""

	@echo -n "  -  MonoSLAM [Davison et al, TPAMI'07] : " ; if [ -d benchmarks/monoslam/src/original ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (make monoslam)\033[0m" ; fi
	@echo    "    repository: https://github.com/hanmekim/SceneLib2"
	@echo    "    available targets are : monoslam"
	@echo    ""

	@echo -n "  - PTAM [Klein et al, ISMAR'07 and ECCV'08] : " ; if [ -d benchmarks/ptam/src/original ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (make ptam)\033[0m" ; fi
	@echo    "    repository: https://github.com/Oxford-PTAM/PTAM-GPL"
	@echo    "    available targets are : ptam"
	@echo    ""

	@echo -n "  - OKVIS [Leutenegger et al, IJRR'15] : " ; if [ -d benchmarks/okvis/src/original ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (make okvis)\033[0m" ; fi
	@echo    "    repository: https://github.com/ethz-asl/okvis"
	@echo    "    available targets are : okvis"
	@echo    ""

	@echo -n "  - SVO [Forster et al, ICRA'14] : " ; if [ -d benchmarks/svo/src/original ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (make svo)\033[0m" ; fi
	@echo    "    repository: https://github.com/uzh-rpg/rpg_svo"
	@echo    "    available targets are : svo"
	@echo    ""

	@echo -n "  - KFusion [Newcombe et al. ISMAR'11] : " ; if [ -d benchmarks/kfusion/src/original ] ; then echo -e "\033[1;32mFound\033[0m" ; else echo -e "\033[1;31mNot found (make kfusion)\033[0m" ; fi
	@echo    "    repository: https://github.com/GerhardR/kfusion"
	@echo    "    available targets are : kfusion"
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
	@echo    "  - ElasticFusion [Whelan et al, IJRR'16] "
	@echo    "    Original repository: https://github.com/mp3guy/ElasticFusion"
	@echo    "    Used repository: https://github.com/bbodin/ElasticFusion"
	@echo "================================================================================================================="
	@echo ""
	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/efusion/src/original
	rm benchmarks/efusion/src/original -rf
	git clone --branch master https://github.com/bbodin/ElasticFusion benchmarks/efusion/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt
infinitam:
	@echo "================================================================================================================="
	@echo    "  - InfiniTAMv2 [Kahler et al, ISMAR'15] "
	@echo    "    Original repository: https://github.com/ethz-asl/infinitam"
	@echo    "    Used repository: https://github.com/bbodin/InfiniTAM.git"
	@echo "================================================================================================================="
	@echo ""
	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/infinitam/src/original
	rm benchmarks/infinitam/src/original -rf
	git clone --branch update-master https://github.com/bbodin/InfiniTAM.git benchmarks/infinitam/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt
lsdslam:
	@echo "================================================================================================================="
	@echo    "  - LSDSLAM [Engel et al, ECCV'14] "
	@echo    "    Original repository: https://github.com/tum-vision/lsd_slam"
	@echo    "    Used repository: https://github.com/pamela-project/lsd_slam.git"
	@echo "================================================================================================================="
	@echo ""
	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/lsdslam/src/original
	rm benchmarks/lsdslam/src/original -rf
	git clone --branch master https://github.com/pamela-project/lsd_slam.git benchmarks/lsdslam/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt
	mkdir -p benchmarks/lsdslam/src/cpp
	rm benchmarks/lsdslam/src/cpp -rf
	git clone --branch cpp https://github.com/pamela-project/lsd_slam.git benchmarks/lsdslam/src/cpp
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt
orbslam2:
	@echo "================================================================================================================="
	@echo    "  - ORBSLAM2 [Mur-Artal et al, TOR'15 and TOR'17] "
	@echo    "    Original repository: https://github.com/raulmur/ORB_SLAM2"
	@echo    "    Used repository: https://github.com/pamela-project/ORB_SLAM2.git"
	@echo "================================================================================================================="
	@echo ""
	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/orbslam2/src/original
	rm benchmarks/orbslam2/src/original -rf
	git clone --branch update-master https://github.com/pamela-project/ORB_SLAM2.git benchmarks/orbslam2/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt
monoslam:
	@echo "================================================================================================================="
	@echo    "  -  MonoSLAM [Davison et al, TPAMI'07] "
	@echo    "    Original repository: https://github.com/hanmekim/SceneLib2"
	@echo    "    Used repository: https://github.com/bbodin/SceneLib2"
	@echo "================================================================================================================="
	@echo ""
	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/monoslam/src/original
	rm benchmarks/monoslam/src/original -rf
	git clone --branch master https://github.com/bbodin/SceneLib2 benchmarks/monoslam/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt
ptam:
	@echo "================================================================================================================="
	@echo    "  - PTAM [Klein et al, ISMAR'07 and ECCV'08] "
	@echo    "    Original repository: https://github.com/Oxford-PTAM/PTAM-GPL"
	@echo    "    Used repository: https://github.com/bbodin/PTAM-GPL"
	@echo "================================================================================================================="
	@echo ""
	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/ptam/src/original
	rm benchmarks/ptam/src/original -rf
	git clone --branch master https://github.com/bbodin/PTAM-GPL benchmarks/ptam/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt
okvis:
	@echo "================================================================================================================="
	@echo    "  - OKVIS [Leutenegger et al, IJRR'15] "
	@echo    "    Original repository: https://github.com/ethz-asl/okvis"
	@echo    "    Used repository: https://github.com/bbodin/okvis"
	@echo "================================================================================================================="
	@echo ""
	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/okvis/src/original
	rm benchmarks/okvis/src/original -rf
	git clone --branch master https://github.com/bbodin/okvis benchmarks/okvis/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt
svo:
	@echo "================================================================================================================="
	@echo    "  - SVO [Forster et al, ICRA'14] "
	@echo    "    Original repository: https://github.com/uzh-rpg/rpg_svo"
	@echo    "    Used repository: https://github.com/pamela-project/rpg_svo.git"
	@echo "================================================================================================================="
	@echo ""
	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/svo/src/original
	rm benchmarks/svo/src/original -rf
	git clone --branch master https://github.com/pamela-project/rpg_svo.git benchmarks/svo/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt
kfusion:
	@echo "================================================================================================================="
	@echo    "  - KFusion [Newcombe et al. ISMAR'11] "
	@echo    "    Original repository: https://github.com/GerhardR/kfusion"
	@echo    "    Used repository: https://github.com/pamela-project/kfusion"
	@echo "================================================================================================================="
	@echo ""
	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/kfusion/src/original
	rm benchmarks/kfusion/src/original -rf
	git clone --branch update-master https://github.com/pamela-project/kfusion benchmarks/kfusion/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt
.PHONY: svo orbslam2 ptam efusion kfusion lsdslam monoslam infinitam okvis
algorithms :  svo orbslam2 ptam efusion kfusion lsdslam monoslam infinitam okvis
benchmarks_status:
	@echo "************ Check-in efusion in benchmarks/efusion/src/original"
	@if [ -d benchmarks/efusion/src/original ] ; then git -C benchmarks/efusion/src/original diff; fi
	@echo "************ Check-in infinitam in benchmarks/infinitam/src/original"
	@if [ -d benchmarks/infinitam/src/original ] ; then git -C benchmarks/infinitam/src/original diff; fi
	@echo "************ Check-in lsdslam in benchmarks/lsdslam/src/original"
	@if [ -d benchmarks/lsdslam/src/original ] ; then git -C benchmarks/lsdslam/src/original diff; fi
	@echo "************ Check-in lsdslam in benchmarks/lsdslam/src/cpp"
	@if [ -d benchmarks/lsdslam/src/cpp ] ; then git -C benchmarks/lsdslam/src/cpp diff; fi
	@echo "************ Check-in orbslam2 in benchmarks/orbslam2/src/original"
	@if [ -d benchmarks/orbslam2/src/original ] ; then git -C benchmarks/orbslam2/src/original diff; fi
	@echo "************ Check-in monoslam in benchmarks/monoslam/src/original"
	@if [ -d benchmarks/monoslam/src/original ] ; then git -C benchmarks/monoslam/src/original diff; fi
	@echo "************ Check-in ptam in benchmarks/ptam/src/original"
	@if [ -d benchmarks/ptam/src/original ] ; then git -C benchmarks/ptam/src/original diff; fi
	@echo "************ Check-in okvis in benchmarks/okvis/src/original"
	@if [ -d benchmarks/okvis/src/original ] ; then git -C benchmarks/okvis/src/original diff; fi
	@echo "************ Check-in svo in benchmarks/svo/src/original"
	@if [ -d benchmarks/svo/src/original ] ; then git -C benchmarks/svo/src/original diff; fi
	@echo "************ Check-in kfusion in benchmarks/kfusion/src/original"
	@if [ -d benchmarks/kfusion/src/original ] ; then git -C benchmarks/kfusion/src/original diff; fi
