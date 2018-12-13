#!/bin/bash

#######################################################################################
#############   READ BASH PARAMETERS
#######################################################################################

echo ""
echo ""
echo ""
echo ""

echo "This script has been execute in " `pwd`
echo BUILD_NUMBER=$BUILD_NUMBER
echo BUILD_ID=$BUILD_ID
echo BUILD_DISPLAY_NAME=$BUILD_DISPLAY_NAME
echo BUILD_TAG=$BUILD_TAG
echo EXECUTOR_NUMBER=$EXECUTOR_NUMBER
echo NODE_NAME=$NODE_NAME
echo NODE_LABELS=$NODE_LABELS
echo WORKSPACE=$WORKSPACE
echo JENKINS_HOME=$JENKINS_HOME
echo JENKINS_URL=$JENKINS_URL
echo BUILD_URL=$BUILD_URL
echo JOB_URL=$JOB_URL
echo SVN_REVISION=$SVN_REVISION
echo SVN_URL=$SVN_URL
echo GIT_REPOSITORY=${GIT_REPOSITORY}
echo GIT_BRANCH=${GIT_BRANCH}

echo ""
echo ""
echo ""
echo ""


#######################################################################################
#############   READ CUSTOM PARAMETERS
#######################################################################################

echo ""
echo ""
echo ""
echo ""

echo SKIP=${SKIP}
echo WORKSPACE=${WORKSPACE}
echo GIT_REPOSITORY=${GIT_REPOSITORY}
echo GIT_BRANCH=${GIT_BRANCH}
echo CLEAN_ALL=${CLEAN_ALL}
echo CLEAN_DATA=${CLEAN_DATA}
echo CLEAN_BUILD=${CLEAN_BUILD}

echo ""
echo ""
echo ""
echo ""

#######################################################################################
#############   START THE REAL DEAL
#######################################################################################


set -x -e


#######################################################################################
#############   CHECK PARAMETERS
#######################################################################################

if [ ! -n "${SKIP+1}" ]; then
  echo "SKIP is not defined."
fi


if [ ! -n "${WORKSPACE+1}" ]; then
  echo "WORKSPACE is not defined !!"
  exit 1
fi


if [ ! -n "${GIT_REPOSITORY+1}" ]; then
  echo "GIT_REPOSITORY is not defined !!"
  exit 1
fi

if [ ! -n "${GIT_BRANCH+1}" ]; then
  echo "GIT_BRANCH is not defined !!"
  exit 1
fi

if [ ! -n "${CLEAN_ALL+1}" ]; then
  echo "CLEAN_ALL is not defined !!"
  exit 1
fi

if [ ! -n "${CLEAN_DATA+1}" ]; then
  echo "CLEAN_DATA is not defined !!"
  exit 1
fi

if [ ! -n "${CLEAN_BUILD+1}" ]; then
  echo "CLEAN_BUILD is not defined !!"
  exit 1
fi





#######################################################################################
#############   DEFINE PARAMETERS
#######################################################################################

LOG_DIRECTORY=${WORKSPACE}/
GIT_LOG_FILE=${LOG_DIRECTORY}/git.log
DEPS_LOG_FILE=${LOG_DIRECTORY}/deps.log
MAKE_LOG_FILE=${LOG_DIRECTORY}/make.log
DATA_LOG_FILE=${LOG_DIRECTORY}/data.log

mkdir -p ${LOG_DIRECTORY}

#######################################################################################
#############   DEFINE CUDA
#######################################################################################


if [ ! -n "${CUDA_TOOLKIT_ROOT_DIR+1}" ]; then
    echo "CUDA_TOOLKIT_ROOT_DIR is not defined."
else
    [ -x ${CUDA_TOOLKIT_ROOT_DIR}/bin/nvcc ]                         && export NVVMIR_LIBRARY_DIR= ${CUDA_TOOLKIT_ROOT_DIR}/share/cuda
    [ -x ${CUDA_TOOLKIT_ROOT_DIR}/libexec/cuda/open64/bin/nvopencc ] && export PATH=$PATH: ${CUDA_TOOLKIT_ROOT_DIR}/libexec/cuda/open64/bin
    [ -d ${CUDA_TOOLKIT_ROOT_DIR}/include/cuda ]                     && export CUDA_INC_PATH= ${CUDA_TOOLKIT_ROOT_DIR}/include/cuda    
fi


#######################################################################################
#############   Cleaning and git step
#######################################################################################

if ! git config --global user.email ; then
    echo "WARNING: CHANGE GIT CONFIG EMAIL"
    git config --global user.email "jenkins@localhost"    
fi

if ! git config --global user.name ; then
    echo "WARNING: CHANGE GIT CONFIG NAME"
    git config --global user.name "jenkins"
fi
  
if [ ! -n "${GIT_LOG_FILE+1}" ]; then
  echo "GIT_LOG_FILE is not defined !!"
  exit 1
fi

rm -f ${GIT_LOG_FILE}

#### CLEAN ALL ####

if [ "${CLEAN_ALL}" == "true" ]; then
    rm  -Rf ${WORKSPACE}/repository
fi


#### CLONE OR UPDATE  ####

if [ ! -x ${WORKSPACE}/repository ] ; then git clone ${GIT_REPOSITORY} ${WORKSPACE}/repository >> ${GIT_LOG_FILE} 2>&1 ; fi


cd ${WORKSPACE}/repository

git remote update  >> ${GIT_LOG_FILE} 2>&1 
git checkout ${GIT_BRANCH}  >> ${GIT_LOG_FILE} 2>&1 
git reset --hard  >> ${GIT_LOG_FILE} 2>&1  
git pull origin ${GIT_BRANCH}  >> ${GIT_LOG_FILE} 2>&1 

#### CLEAN INSIDE  ####

if [ "${CLEAN_BUILD}" == "true" ]; then	
    make clean
fi

if [ "${CLEAN_DATA}" == "true" ]; then	
    make cleandatasets
fi



#######################################################################################
#############   Make dependencies 
#######################################################################################


if [ ! -n "${DEPS_LOG_FILE+1}" ]; then
  echo "DEPS_LOG_FILE is not defined !!"
  exit 1
fi

rm -f ${DEPS_LOG_FILE}
make gcc7       >> ${DEPS_LOG_FILE} 2>&1
make deps       >> ${DEPS_LOG_FILE} 2>&1 

#######################################################################################
#############   Make slambench and algorithms
#######################################################################################

if [ ! -n "${MAKE_LOG_FILE+1}" ]; then
  echo "MAKE_LOG_FILE is not defined !!"
  exit 1
fi

rm -f ${MAKE_LOG_FILE}
make algorithms SBQUIET=1           >> ${MAKE_LOG_FILE} 
VERBOSE=1 make slambench APPS=all   >> ${MAKE_LOG_FILE} 2>&1 



#######################################################################################
#############   Make datasets
#######################################################################################

if [ ! -n "${DATA_LOG_FILE+1}" ]; then
  echo "DATA_LOG_FILE is not defined !!"
  exit 1
fi

rm -f ${DATA_LOG_FILE}


### ORBSLAM Vocabulary Dataset
make ./benchmarks/orbslam2/src/original/Vocabulary/ORBvoc.txt   >> ${DATA_LOG_FILE} 2>&1 

### ICL-NUIM Living Room
     
make ./datasets/ICL_NUIM/living_room_traj0_loop.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/ICL_NUIM/living_room_traj1_loop.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/ICL_NUIM/living_room_traj2_loop.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/ICL_NUIM/living_room_traj3_loop.slam   >> ${DATA_LOG_FILE} 2>&1 


### TUM Testing and Debugging

make ./datasets/TUM/freiburg1/rgbd_dataset_freiburg1_xyz.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/TUM/freiburg1/rgbd_dataset_freiburg1_rpy.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_xyz.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_rpy.slam   >> ${DATA_LOG_FILE} 2>&1 


### TUM Handheld SLAM

make ./datasets/TUM/freiburg1/rgbd_dataset_freiburg1_360.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/TUM/freiburg1/rgbd_dataset_freiburg1_floor.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/TUM/freiburg1/rgbd_dataset_freiburg1_desk.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/TUM/freiburg1/rgbd_dataset_freiburg1_desk2.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/TUM/freiburg1/rgbd_dataset_freiburg1_room.slam   >> ${DATA_LOG_FILE} 2>&1 

make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_360_hemisphere.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_360_kidnap.slam   >> ${DATA_LOG_FILE} 2>&1 

make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_desk.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_desk_with_person.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_large_no_loop.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_large_with_loop.slam   >> ${DATA_LOG_FILE} 2>&1 

### TUM Robot SLAM

make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_pioneer_360.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_pioneer_slam.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_pioneer_slam2.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_pioneer_slam3.slam   >> ${DATA_LOG_FILE} 2>&1 

##### TUM Extra dataset

## https://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household.tgz


### EuRoC MAV Machine Hall

make ./datasets/EuRoCMAV/machine_hall/MH_01_easy/MH_01_easy.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/EuRoCMAV/machine_hall/MH_02_easy/MH_02_easy.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/EuRoCMAV/machine_hall/MH_03_medium/MH_03_medium.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/EuRoCMAV/machine_hall/MH_04_difficult/MH_04_difficult.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/EuRoCMAV/machine_hall/MH_05_difficult/MH_05_difficult.slam   >> ${DATA_LOG_FILE} 2>&1 

### EuRoC MAV Vicon Room

make ./datasets/EuRoCMAV/vicon_room1/V1_01_easy/V1_01_easy.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/EuRoCMAV/vicon_room1/V1_02_medium/V1_02_medium.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/EuRoCMAV/vicon_room1/V1_03_difficult/V1_03_difficult.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/EuRoCMAV/vicon_room2/V2_01_easy/V2_01_easy.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/EuRoCMAV/vicon_room2/V2_02_medium/V2_02_medium.slam   >> ${DATA_LOG_FILE} 2>&1 
make ./datasets/EuRoCMAV/vicon_room2/V2_03_difficult/V2_03_difficult.slam   >> ${DATA_LOG_FILE} 2>&1 


### SVO test dataset

make datasets/SVO/artificial.slam   >> ${DATA_LOG_FILE} 2>&1 

echo "End build."

exit 0
