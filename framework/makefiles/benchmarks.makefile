usecases:
	@echo  ""
	@echo  "================================================================================================================="
	@echo -e "Current list of compatible SLAM systems (alphabetical order). If you are using any of the following SLAM algorithms, ${BoldRed} please refer to their respective publications ${ColorOff}"
	@echo  ""
	@echo -n "  - MonoSLAM [Davison et al, TPAMI'07] : " ; if [ -d benchmarks/monoslam/src/original ] ; then echo -e " ${BoldGreen}Found${ColorOff}"; else echo -e " ${BoldRed}Not found (make monoslam) ${ColorOff}" ; fi
	@echo  "      Original repository: https://github.com/hanmekim/SceneLib2"
	@echo  "      Used repository: https://github.com/bbodin/SceneLib2"
	@echo  "      available targets are : monoslam"
	@echo  ""

	@echo -n "  - PTAM [Klein et al, ISMAR'07 and ECCV'08] : " ; if [ -d benchmarks/ptam/src/original ] ; then echo -e " ${BoldGreen}Found${ColorOff}"; else echo -e " ${BoldRed}Not found (make ptam) ${ColorOff}" ; fi
	@echo  "      Original repository: https://github.com/Oxford-PTAM/PTAM-GPL"
	@echo  "      Used repository: https://github.com/bbodin/PTAM-GPL"
	@echo  "      available targets are : ptam"
	@echo  ""

	@echo -n "  - KFusion [Newcombe et al. ISMAR'11] (Gerhard Reitmayr's implementation, old) : " ; if [ -d benchmarks/kfusion/src/original ] ; then echo -e " ${BoldGreen}Found${ColorOff}"; else echo -e " ${BoldRed}Not found (make kfusion) ${ColorOff}" ; fi
	@echo  "      Original repository: https://github.com/GerhardR/kfusion"
	@echo  "      Used repository: https://github.com/pamela-project/kfusion"
	@echo  "      available targets are : kfusion"
	@echo  ""

	@echo -n "  - KinectFusion [Newcombe et al. ISMAR'11] (Christian Diller's implementation, new) : " ; if [ -d benchmarks/kinectfusion/src/original ] ; then echo -e " ${BoldGreen}Found${ColorOff}"; else echo -e " ${BoldRed}Not found (make kinectfusion) ${ColorOff}" ; fi
	@echo  "      Original repository: https://github.com/chrdiller/KinectFusionApp"
	@echo  "      Used repository: https://github.com/mihaibujanca/KinectFusion"
	@echo  "      available targets are : kinectfusion"
	@echo  ""

	@echo -n "  - LSDSLAM [Engel et al, ECCV'14] : " ; if [ -d benchmarks/lsdslam/src/original ] ; then echo -e " ${BoldGreen}Found${ColorOff}"; else echo -e " ${BoldRed}Not found (make lsdslam) ${ColorOff}" ; fi
	@echo  "      Original repository: https://github.com/tum-vision/lsd_slam"
	@echo  "      Used repository: https://github.com/pamela-project/lsd_slam.git"
	@echo  "      available targets are : lsdslam"
	@echo  ""

	@echo -n "  - LSDSLAM [Engel et al, ECCV'14] : " ; if [ -d benchmarks/lsdslam/src/cpp ] ; then echo -e " ${BoldGreen}Found${ColorOff}"; else echo -e " ${BoldRed}Not found (make lsdslam) ${ColorOff}" ; fi
	@echo  "      Original repository: https://github.com/tum-vision/lsd_slam"
	@echo  "      Used repository: https://github.com/pamela-project/lsd_slam.git"
	@echo  "      available targets are : lsdslam"
	@echo  ""

	@echo -n "  - SVO [Forster et al, ICRA'14] : " ; if [ -d benchmarks/svo/src/original ] ; then echo -e " ${BoldGreen}Found${ColorOff}"; else echo -e " ${BoldRed}Not found (make svo) ${ColorOff}" ; fi
	@echo  "      Original repository: https://github.com/uzh-rpg/rpg_svo"
	@echo  "      Used repository: https://github.com/pamela-project/rpg_svo.git"
	@echo  "      available targets are : svo"
	@echo  ""

	@echo -n "  - InfiniTAMv2 [Kahler et al, ISMAR'15] : " ; if [ -d benchmarks/infinitam/src/original ] ; then echo -e " ${BoldGreen}Found${ColorOff}"; else echo -e " ${BoldRed}Not found (make infinitam) ${ColorOff}" ; fi
	@echo  "      Original repository: https://github.com/victorprad/InfiniTAM"
	@echo  "      Used repository: https://github.com/bbodin/InfiniTAM.git"
	@echo  "      available targets are : infinitam"
	@echo  ""

	@echo -n "  - ORB-SLAM2 [Mur-Artal et al, TOR'15 and TOR'17] : " ; if [ -d benchmarks/orbslam2/src/original ] ; then echo -e " ${BoldGreen}Found${ColorOff}"; else echo -e " ${BoldRed}Not found (make orbslam2) ${ColorOff}" ; fi
	@echo  "      Original repository: https://github.com/raulmur/ORB_SLAM2"
	@echo  "      Used repository: https://github.com/pamela-project/ORB_SLAM2.git"
	@echo  "      available targets are : orbslam2"
	@echo  ""

	@echo -n "  - OKVIS [Leutenegger et al, IJRR'15] : " ; if [ -d benchmarks/okvis/src/original ] ; then echo -e " ${BoldGreen}Found${ColorOff}"; else echo -e " ${BoldRed}Not found (make okvis) ${ColorOff}" ; fi
	@echo  "      Original repository: https://github.com/ethz-asl/okvis"
	@echo  "      Used repository: https://github.com/bbodin/okvis"
	@echo  "      available targets are : okvis"
	@echo  ""

	@echo -n "  - DSO [Engel et al. Arxiv'16] : " ; if [ -d benchmarks/dso/src/original ] ; then echo -e " ${BoldGreen}Found${ColorOff}"; else echo -e " ${BoldRed}Not found (make dso) ${ColorOff}" ; fi
	@echo  "      Original repository: https://github.com/JakobEngel/dso"
	@echo  "      Used repository: https://github.com/mihaibujanca/dso"
	@echo  "      available targets are : dso"
	@echo  ""

	@echo -n "  - ElasticFusion [Whelan et al, IJRR'16] : " ; if [ -d benchmarks/efusion/src/original ] ; then echo -e " ${BoldGreen}Found${ColorOff}"; else echo -e " ${BoldRed}Not found (make efusion) ${ColorOff}" ; fi
	@echo  "      Original repository: https://github.com/mp3guy/ElasticFusion"
	@echo  "      Used repository: https://github.com/bbodin/ElasticFusion"
	@echo  "      available targets are : efusion"
	@echo  ""

	@echo -n "  - BundleFusion [Dai et al. ACM TOG'17] : " ; if [ -d benchmarks/bundlefusion/src/original ] ; then echo -e " ${BoldGreen}Found${ColorOff}"; else echo -e " ${BoldRed}Not found (make bundlefusion) ${ColorOff}" ; fi
	@echo  "      Original repository: https://github.com/niessner/BundleFusion"
	@echo  "      Used repository: https://github.com/Paul92/BundleFusion"
	@echo  "      available targets are : bundlefusion"
	@echo  ""

	@echo -n "  - SemanticFusion [McCormac et al. ICRA'17] : " ; if [ -d benchmarks/semanticfusion/src/original ] ; then echo -e " ${BoldGreen}Found${ColorOff}"; else echo -e " ${BoldRed}Not found (make semanticfusion) ${ColorOff}" ; fi
	@echo  "      Original repository: https://github.com/seaun163/semanticfusion"
	@echo  "      Used repository: https://github.com/Paul92/semanticfusion"
	@echo  "      available targets are : semanticfusion"
	@echo  ""

	@echo -n "  - FLAME [Greene et al. ICCV'17] : " ; if [ -d benchmarks/flame/src/original ] ; then echo -e " ${BoldGreen}Found${ColorOff}"; else echo -e " ${BoldRed}Not found (make flame) ${ColorOff}" ; fi
	@echo  "      Original repository: https://github.com/robustrobotics/flame"
	@echo  "      Used repository: https://github.com/mihaibujanca/flame"
	@echo  "      available targets are : flame"
	@echo  ""

	@echo -n "  - Supereight [Vespa et al. RA-L'18] : " ; if [ -d benchmarks/supereight/src/original ] ; then echo -e " ${BoldGreen}Found${ColorOff}"; else echo -e " ${BoldRed}Not found (make supereight) ${ColorOff}" ; fi
	@echo  "      Original repository: https://github.com/emanuelev/supereight"
	@echo  "      Used repository: https://github.com/pamela-project/supereight"
	@echo  "      available targets are : supereight"
	@echo  ""

	@echo -n "  - ReFusion [Palazollo et al. IROS'19] : " ; if [ -d benchmarks/refusion/src/original ] ; then echo -e " ${BoldGreen}Found${ColorOff}"; else echo -e " ${BoldRed}Not found (make refusion) ${ColorOff}" ; fi
	@echo  "      Original repository: https://github.com/PRBonn/refusion"
	@echo  "      Used repository: https://github.com/mihaibujanca/refusion"
	@echo  "      available targets are : refusion"
	@echo  ""

	@echo -n "  - OpenVINS [Geneva et al. IROS'19] : " ; if [ -d benchmarks/open_vins/src/original ] ; then echo -e " ${BoldGreen}Found${ColorOff}"; else echo -e " ${BoldRed}Not found (make open_vins) ${ColorOff}" ; fi
	@echo  "      Original repository: https://github.com/rpng/open_vins"
	@echo  "      Used repository: https://github.com/mihaibujanca/open_vins"
	@echo  "      available targets are : open_vins"
	@echo  ""

	@echo -n "  - ORB-SLAM3 [Campos et al, ARXIV'20] : " ; if [ -d benchmarks/orbslam3/src/original ] ; then echo -e " ${BoldGreen}Found${ColorOff}"; else echo -e " ${BoldRed}Not found (make orbslam3) ${ColorOff}" ; fi
	@echo  "      Original repository: https://github.com/UZ-SLAMLab/ORB_SLAM3"
	@echo  "      Used repository: https://github.com/mihaibujanca/ORB_SLAM3"
	@echo  "      available targets are : orbslam3"
	@echo  ""

	@echo -n "  - FullFusion [Bujanca et al. ICCVW'19] : " ; if [ -d benchmarks/fullfusion/src/original ] ; then echo -e " ${BoldGreen}Found${ColorOff}"; else echo -e " ${BoldRed}Not found (make fullfusion) ${ColorOff}" ; fi
	@echo  "      Original repository: https://github.com/mihaibujanca/fullfusion-jetson"
	@echo  "      Used repository: https://github.com/mihaibujanca/fullfusion-jetson"
	@echo  "      available targets are : fullfusion"
	@echo  ""

	@echo  "If you want to test SLAMBench with existing SLAM algorithms, once you have download it please run \"make slambench APPS=slam1,slam2,...\""
	@echo  "   e.g. make slambench APPS=kfusion,orbslam2"
	@echo  "   You can also use \"make slambench APPS=all\" to compile them all."
	@echo  ""
	@echo  "As a next step we suggest you run make datasets."
	@echo  ""
	@echo  "================================================================================================================="

monoslam:
	@echo  "================================================================================================================="
	@echo  "MonoSLAM [Davison et al, TPAMI'07]"
	@echo  "Original repository: https://github.com/hanmekim/SceneLib2"
	@echo  "Used repository: https://github.com/bbodin/SceneLib2"
	@echo  "================================================================================================================="

	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/monoslam/src/original
	rm benchmarks/monoslam/src/original -rf
	git clone --recursive --branch master https://github.com/bbodin/SceneLib2 benchmarks/monoslam/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

ptam:
	@echo  "================================================================================================================="
	@echo  "PTAM [Klein et al, ISMAR'07 and ECCV'08]"
	@echo  "Original repository: https://github.com/Oxford-PTAM/PTAM-GPL"
	@echo  "Used repository: https://github.com/bbodin/PTAM-GPL"
	@echo  "================================================================================================================="

	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/ptam/src/original
	rm benchmarks/ptam/src/original -rf
	git clone --recursive --branch master https://github.com/bbodin/PTAM-GPL benchmarks/ptam/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

kfusion:
	@echo  "================================================================================================================="
	@echo  "KFusion [Newcombe et al. ISMAR'11] (Gerhard Reitmayr's implementation, old)"
	@echo  "Original repository: https://github.com/GerhardR/kfusion"
	@echo  "Used repository: https://github.com/pamela-project/kfusion"
	@echo  "================================================================================================================="

	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/kfusion/src/original
	rm benchmarks/kfusion/src/original -rf
	git clone --recursive --branch update-master https://github.com/pamela-project/kfusion benchmarks/kfusion/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

kinectfusion:
	@echo  "================================================================================================================="
	@echo  "KinectFusion [Newcombe et al. ISMAR'11] (Christian Diller's implementation, new)"
	@echo  "Original repository: https://github.com/chrdiller/KinectFusionApp"
	@echo  "Used repository: https://github.com/mihaibujanca/KinectFusion"
	@echo  "================================================================================================================="

	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/kinectfusion/src/original
	rm benchmarks/kinectfusion/src/original -rf
	git clone --recursive --branch master https://github.com/mihaibujanca/KinectFusion benchmarks/kinectfusion/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

lsdslam:
	@echo  "================================================================================================================="
	@echo  "LSDSLAM [Engel et al, ECCV'14]"
	@echo  "Original repository: https://github.com/tum-vision/lsd_slam"
	@echo  "Used repository: https://github.com/pamela-project/lsd_slam.git"
	@echo  "================================================================================================================="

	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/lsdslam/src/original
	rm benchmarks/lsdslam/src/original -rf
	git clone --recursive --branch master https://github.com/pamela-project/lsd_slam.git benchmarks/lsdslam/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt
	mkdir -p benchmarks/lsdslam/src/cpp
	rm benchmarks/lsdslam/src/cpp -rf
	git clone --recursive --branch cpp https://github.com/pamela-project/lsd_slam.git benchmarks/lsdslam/src/cpp
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

svo:
	@echo  "================================================================================================================="
	@echo  "SVO [Forster et al, ICRA'14]"
	@echo  "Original repository: https://github.com/uzh-rpg/rpg_svo"
	@echo  "Used repository: https://github.com/pamela-project/rpg_svo.git"
	@echo  "================================================================================================================="

	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/svo/src/original
	rm benchmarks/svo/src/original -rf
	git clone --recursive --branch master https://github.com/pamela-project/rpg_svo.git benchmarks/svo/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

infinitam:
	@echo  "================================================================================================================="
	@echo  "InfiniTAMv2 [Kahler et al, ISMAR'15]"
	@echo  "Original repository: https://github.com/victorprad/InfiniTAM"
	@echo  "Used repository: https://github.com/bbodin/InfiniTAM.git"
	@echo  "================================================================================================================="

	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/infinitam/src/original
	rm benchmarks/infinitam/src/original -rf
	git clone --recursive --branch update-master https://github.com/bbodin/InfiniTAM.git benchmarks/infinitam/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

orbslam2:
	@echo  "================================================================================================================="
	@echo  "ORB-SLAM2 [Mur-Artal et al, TOR'15 and TOR'17]"
	@echo  "Original repository: https://github.com/raulmur/ORB_SLAM2"
	@echo  "Used repository: https://github.com/pamela-project/ORB_SLAM2.git"
	@echo  "================================================================================================================="

	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/orbslam2/src/original
	rm benchmarks/orbslam2/src/original -rf
	git clone --recursive --branch update-master https://github.com/pamela-project/ORB_SLAM2.git benchmarks/orbslam2/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

okvis:
	@echo  "================================================================================================================="
	@echo  "OKVIS [Leutenegger et al, IJRR'15]"
	@echo  "Original repository: https://github.com/ethz-asl/okvis"
	@echo  "Used repository: https://github.com/bbodin/okvis"
	@echo  "================================================================================================================="

	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/okvis/src/original
	rm benchmarks/okvis/src/original -rf
	git clone --recursive --branch master https://github.com/bbodin/okvis benchmarks/okvis/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

dso:
	@echo  "================================================================================================================="
	@echo  "DSO [Engel et al. Arxiv'16]"
	@echo  "Original repository: https://github.com/JakobEngel/dso"
	@echo  "Used repository: https://github.com/mihaibujanca/dso"
	@echo  "================================================================================================================="

	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/dso/src/original
	rm benchmarks/dso/src/original -rf
	git clone --recursive --branch master https://github.com/mihaibujanca/dso benchmarks/dso/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

efusion:
	@echo  "================================================================================================================="
	@echo  "ElasticFusion [Whelan et al, IJRR'16]"
	@echo  "Original repository: https://github.com/mp3guy/ElasticFusion"
	@echo  "Used repository: https://github.com/bbodin/ElasticFusion"
	@echo  "================================================================================================================="

	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/efusion/src/original
	rm benchmarks/efusion/src/original -rf
	git clone --recursive --branch master https://github.com/bbodin/ElasticFusion benchmarks/efusion/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

bundlefusion:
	@echo  "================================================================================================================="
	@echo  "BundleFusion [Dai et al. ACM TOG'17]"
	@echo  "Original repository: https://github.com/niessner/BundleFusion"
	@echo  "Used repository: https://github.com/Paul92/BundleFusion"
	@echo  "================================================================================================================="

	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/bundlefusion/src/original
	rm benchmarks/bundlefusion/src/original -rf
	git clone --recursive --branch master https://github.com/Paul92/BundleFusion benchmarks/bundlefusion/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

semanticfusion:
	@echo  "================================================================================================================="
	@echo  "SemanticFusion [McCormac et al. ICRA'17]"
	@echo  "Original repository: https://github.com/seaun163/semanticfusion"
	@echo  "Used repository: https://github.com/Paul92/semanticfusion"
	@echo  "================================================================================================================="

	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/semanticfusion/src/original
	rm benchmarks/semanticfusion/src/original -rf
	git clone --recursive --branch master https://github.com/Paul92/semanticfusion benchmarks/semanticfusion/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

flame:
	@echo  "================================================================================================================="
	@echo  "FLAME [Greene et al. ICCV'17]"
	@echo  "Original repository: https://github.com/robustrobotics/flame"
	@echo  "Used repository: https://github.com/mihaibujanca/flame"
	@echo  "================================================================================================================="

	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/flame/src/original
	rm benchmarks/flame/src/original -rf
	git clone --recursive --branch master https://github.com/mihaibujanca/flame benchmarks/flame/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

supereight:
	@echo  "================================================================================================================="
	@echo  "Supereight [Vespa et al. RA-L'18]"
	@echo  "Original repository: https://github.com/emanuelev/supereight"
	@echo  "Used repository: https://github.com/pamela-project/supereight"
	@echo  "================================================================================================================="

	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/supereight/src/original
	rm benchmarks/supereight/src/original -rf
	git clone --recursive --branch master https://github.com/pamela-project/supereight benchmarks/supereight/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

refusion:
	@echo  "================================================================================================================="
	@echo  "ReFusion [Palazollo et al. IROS'19]"
	@echo  "Original repository: https://github.com/PRBonn/refusion"
	@echo  "Used repository: https://github.com/mihaibujanca/refusion"
	@echo  "================================================================================================================="

	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/refusion/src/original
	rm benchmarks/refusion/src/original -rf
	git clone --recursive --branch master https://github.com/mihaibujanca/refusion benchmarks/refusion/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

open_vins:
	@echo  "================================================================================================================="
	@echo  "OpenVINS [Geneva et al. IROS'19]"
	@echo  "Original repository: https://github.com/rpng/open_vins"
	@echo  "Used repository: https://github.com/mihaibujanca/open_vins"
	@echo  "================================================================================================================="

	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/open_vins/src/original
	rm benchmarks/open_vins/src/original -rf
	git clone --recursive --branch rpng-master https://github.com/mihaibujanca/open_vins benchmarks/open_vins/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

orbslam3:
	@echo  "================================================================================================================="
	@echo  "ORB-SLAM3 [Campos et al, ARXIV'20]"
	@echo  "Original repository: https://github.com/UZ-SLAMLab/ORB_SLAM3"
	@echo  "Used repository: https://github.com/mihaibujanca/ORB_SLAM3"
	@echo  "================================================================================================================="

	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/orbslam3/src/original
	rm benchmarks/orbslam3/src/original -rf
	git clone --recursive --branch master https://github.com/mihaibujanca/ORB_SLAM3 benchmarks/orbslam3/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

fullfusion:
	@echo  "================================================================================================================="
	@echo  "FullFusion [Bujanca et al. ICCVW'19]"
	@echo  "Original repository: https://github.com/mihaibujanca/fullfusion-jetson"
	@echo  "Used repository: https://github.com/mihaibujanca/fullfusion-jetson"
	@echo  "================================================================================================================="

	@echo "Are you sure you want to download this use-case (y/n) ?" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! "$$REPLY" == "y" ] ; then echo -e "\nExit."; false; else echo -e "\nDownload starts."; fi
	mkdir -p benchmarks/fullfusion/src/original
	rm benchmarks/fullfusion/src/original -rf
	git clone --recursive --branch master https://github.com/mihaibujanca/fullfusion-jetson benchmarks/fullfusion/src/original
	@echo "cmake_minimum_required(VERSION 2.8)"   > benchmarks/$@/CMakeLists.txt
	@echo "explore_implementations ( $@ src/* )"     >> benchmarks/$@/CMakeLists.txt

.PHONY: okvis fullfusion orbslam2 svo open_vins ptam efusion refusion dso orbslam3 semanticfusion supereight bundlefusion infinitam monoslam lsdslam kinectfusion flame kfusion
algorithms :  okvis fullfusion orbslam2 svo open_vins ptam efusion refusion dso orbslam3 semanticfusion supereight bundlefusion infinitam monoslam lsdslam kinectfusion flame kfusion

benchmarks_status:
	@echo  "************ Check-in monoslam in benchmarks/monoslam/src/original"
	@if [ -d benchmarks/monoslam/src/original ] ; then git -C benchmarks/monoslam/src/original diff; fi
	@echo  "************ Check-in ptam in benchmarks/ptam/src/original"
	@if [ -d benchmarks/ptam/src/original ] ; then git -C benchmarks/ptam/src/original diff; fi
	@echo  "************ Check-in kfusion in benchmarks/kfusion/src/original"
	@if [ -d benchmarks/kfusion/src/original ] ; then git -C benchmarks/kfusion/src/original diff; fi
	@echo  "************ Check-in kinectfusion in benchmarks/kinectfusion/src/original"
	@if [ -d benchmarks/kinectfusion/src/original ] ; then git -C benchmarks/kinectfusion/src/original diff; fi
	@echo  "************ Check-in lsdslam in benchmarks/lsdslam/src/original"
	@if [ -d benchmarks/lsdslam/src/original ] ; then git -C benchmarks/lsdslam/src/original diff; fi
	@echo  "************ Check-in lsdslam in benchmarks/lsdslam/src/cpp"
	@if [ -d benchmarks/lsdslam/src/cpp ] ; then git -C benchmarks/lsdslam/src/cpp diff; fi
	@echo  "************ Check-in svo in benchmarks/svo/src/original"
	@if [ -d benchmarks/svo/src/original ] ; then git -C benchmarks/svo/src/original diff; fi
	@echo  "************ Check-in infinitam in benchmarks/infinitam/src/original"
	@if [ -d benchmarks/infinitam/src/original ] ; then git -C benchmarks/infinitam/src/original diff; fi
	@echo  "************ Check-in orbslam2 in benchmarks/orbslam2/src/original"
	@if [ -d benchmarks/orbslam2/src/original ] ; then git -C benchmarks/orbslam2/src/original diff; fi
	@echo  "************ Check-in okvis in benchmarks/okvis/src/original"
	@if [ -d benchmarks/okvis/src/original ] ; then git -C benchmarks/okvis/src/original diff; fi
	@echo  "************ Check-in dso in benchmarks/dso/src/original"
	@if [ -d benchmarks/dso/src/original ] ; then git -C benchmarks/dso/src/original diff; fi
	@echo  "************ Check-in efusion in benchmarks/efusion/src/original"
	@if [ -d benchmarks/efusion/src/original ] ; then git -C benchmarks/efusion/src/original diff; fi
	@echo  "************ Check-in bundlefusion in benchmarks/bundlefusion/src/original"
	@if [ -d benchmarks/bundlefusion/src/original ] ; then git -C benchmarks/bundlefusion/src/original diff; fi
	@echo  "************ Check-in semanticfusion in benchmarks/semanticfusion/src/original"
	@if [ -d benchmarks/semanticfusion/src/original ] ; then git -C benchmarks/semanticfusion/src/original diff; fi
	@echo  "************ Check-in flame in benchmarks/flame/src/original"
	@if [ -d benchmarks/flame/src/original ] ; then git -C benchmarks/flame/src/original diff; fi
	@echo  "************ Check-in supereight in benchmarks/supereight/src/original"
	@if [ -d benchmarks/supereight/src/original ] ; then git -C benchmarks/supereight/src/original diff; fi
	@echo  "************ Check-in refusion in benchmarks/refusion/src/original"
	@if [ -d benchmarks/refusion/src/original ] ; then git -C benchmarks/refusion/src/original diff; fi
	@echo  "************ Check-in open_vins in benchmarks/open_vins/src/original"
	@if [ -d benchmarks/open_vins/src/original ] ; then git -C benchmarks/open_vins/src/original diff; fi
	@echo  "************ Check-in orbslam3 in benchmarks/orbslam3/src/original"
	@if [ -d benchmarks/orbslam3/src/original ] ; then git -C benchmarks/orbslam3/src/original diff; fi
	@echo  "************ Check-in fullfusion in benchmarks/fullfusion/src/original"
	@if [ -d benchmarks/fullfusion/src/original ] ; then git -C benchmarks/fullfusion/src/original diff; fi
