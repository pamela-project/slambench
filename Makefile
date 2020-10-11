ECHO=/bin/echo
#SHELL := /bin/bash
WGET:=wget
GET_REPLY:=read REPLY
ifeq ("${SBQUIET}","1")
GET_REPLY:=REPLY="y"
WGET:=wget -q
endif

BoldGreen=\033[1;32m
BoldRed=\033[1;31m
ColorOff=\033[0m

#### SLAMBENCH INFOS ####
infos:
	@${ECHO} -e "\n*** SLAMBench is an evaluation framework for SLAM algorithms. *** "
	@${ECHO} -e "\n\
	  (1) First, several dependencies are needed to compile SLAMBench and its use-cases.\n\
	      We suggest that you to download and install them automatically using the following command:\n\
     ${BoldGreen} - make deps ${ColorOff}\n\
	  (2) Then, to compile the SLAMBench framework you just need to type:\n\
	     ${BoldGreen} - make slambench ${ColorOff}\n\
	  (3) SLAMBench integrates a number of algorithms, however they are not directly distribute with the framework\n\
	      To download and build all the available algorithms, use:\n\
	     ${BoldGreen} - make algorithms${ColorOff}\n\
	      Alternatively, to see the list of available algorithms, use:\n\
	     ${BoldGreen} - make usecases ${ColorOff}\n\
	  (4) For information about downloading and building one of the available datasets, use:\n\
	     ${BoldGreen} - make datasetslist ${ColorOff}\n\
	  (5) Once the desired datasets and algorithms were built, you can run the benchmark using:\n\
	     ${BoldGreen} - ./build/bin/slambench -i path/to/dataset.slam -load path/to/algorithm-library.so ${ColorOff}\n\
	"

#### Dependencies ####
include ./framework/makefiles/deps.makefile

#### Compilation targets ####
build/Makefile : framework/CMakeLists.txt
	mkdir -p build/
	cd build; ${DEPS_ENV} cmake -U -Bbuild -H.  -DAPPS="${APPS}"  ${DEPS_ARGS} -D"CMAKE_MODULE_PATH:PATH=${ROOT_DIR}/cmake_modules" ..

slambench: build/Makefile
	$(MAKE) -C build $(MFLAGS)
	@echo ""
	@echo "================================================================================================================="
	@echo "The SLAMBench library should have been compiled, you will find binaries in build/bin, and libraries in build/lib."
	@echo ""
	@echo "Tools/Debugger available: "
	@echo -n "  - build/bin/slambench:          " ; if [ -f build/bin/slambench ] ; then echo -e "${BoldGreen}Found${ColorOff}" ; else echo -e "${BoldRed}Not found (Missing dependencies?)${ColorOff}" ; fi
	@echo -n "  - build/bin/pointcloud_aligner: " ; if [ -f build/bin/pointcloud_aligner ] ; then echo -e "${BoldGreen}Found${ColorOff}" ; else echo -e "${BoldRed}Not found (Missing dependencies (i.e. pcl)?)${ColorOff}" ; fi
	@echo -n "  - build/bin/dataset-generator:  " ; if [ -f build/bin/dataset-generator ] ; then echo -e "${BoldGreen}Found${ColorOff}" ; else echo -e "${BoldRed}Not found (Missing dependencies?)${ColorOff}" ; fi
	@echo -n "  - build/bin/io-inspect-file:    " ; if [ -f build/bin/io-inspect-file ] ; then echo -e "${BoldGreen}Found${ColorOff}" ; else echo -e "${BoldRed}Not found (Missing dependencies?)${ColorOff}" ; fi
	@echo -n "  - build/bin/io-readply:         " ; if [ -f build/bin/io-readply ] ; then echo -e "${BoldGreen}Found${ColorOff}" ; else echo -e "${BoldRed}Not found (Missing dependencies?)${ColorOff}" ; fi
	@echo ""
	@echo "The list of current algorithms available is:"
	@echo ""                                                                      
	@for f in `ls build/lib/lib*-library.so 2> /dev/null || echo Nothing` ; do echo $$f ; done
	@echo ""
	@echo "The list of current filters available is:"
	@echo ""
	@for f in `ls build/lib/lib*-filter.so 2> /dev/null || echo Nothing` ; do echo $$f ; done
	@echo ""
	@echo "As a next step we suggest you to run \"make usecases\" or \"make slambench APPS=all\"."

.PHONY: build/Makefile

#### Benchmarks ####
framework/makefiles/benchmarks.makefile : framework/makefiles/download_benchmarks.py benchmarks/benchmarks.repos
	python $^ > $@
include framework/makefiles/benchmarks.makefile

#### Datasets ####
framework/makefiles/datasets.makefile : framework/makefiles/download_datasets.py datasets/datasets.repos
	python $^ > $@
include framework/makefiles/dataset-utils.makefile
include framework/makefiles/datasets.makefile

.PHONY: slambench benchmarks datasets

#### ORBSLAM Voc ####
./benchmarks/orbslam2/src/original/Vocabulary/ORBvoc.txt : ./benchmarks/orbslam2/src/original/Vocabulary/ORBvoc.txt.tar.gz
	cd ./benchmarks/orbslam2/src/original/Vocabulary/ && tar -xf ORBvoc.txt.tar.gz

#### CLEAN TOOLS ####
clean :
	rm -rf build install android-build android-install docker/tmp/

cleandatasets :
	find datasets/ | grep [.]slam | xargs rm 2> /dev/null || true
	@echo "Datasets cleaned"

cleandeps :
	rm -rf deps

cleanall : clean cleandatasets cleandeps

.PHONY: clean cleandeps cleandatasets cleanall

#### DEMO PART ####
demo-prepare :
	make slambench APPS=all
	make datasets/ICL_NUIM/living_room_traj2_loop.slam
	make datasets/EuRoCMAV/machine_hall/MH_01_easy/MH_01_easy.slam
	make datasets/TUM/freiburg2/rgbd_dataset_freiburg2_desk.slam

demo-droneflight :
	$(MAKE) open_vins $(MFLAGS)
	$(MAKE) slambench APPS=open_vins $(MFLAGS)
	$(MAKE) datasets/UZHFPV/indoor_forward_3_snapdragon_with_gt.slam $(MFLAGS)
	@echo "You can now run ${BoldGreen}./build/bin/slambench -i datasets/UZHFPV/indoor_forward_3_snapdragon_with_gt.slam -load build/lib/libopen_vins-original-library.so${ColorOff}"

demo-lib :
	clear
	@${ECHO} -e "                        "
	@${ECHO} -e " ======================="
	@${ECHO} -e " Pick gui and whether to use lifelong SLAM mode: "
	@${ECHO} -e " ./build/bin/slambench -gui <false|true> -lifelong <false|true>"
	@${ECHO} -e " ======================="
	@${ECHO} -e " Pick dataset           "
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

.PHONY: demo-droneflight demo-lib demo-prepare
#### TESTS ####
regression_test :
	make -C docker fastCI-docker-image # First test compilation of slambench and run kfusion
	make -C docker ubuntu-14.04-docker-image # Second compilation for embedded systems such as Odroid or TK1
# third run slambench for multiple datasets and check ATE and Memory usage

test :
	make -C docker fastCI-docker-image
