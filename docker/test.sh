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
echo JOB_NAME=$JOB_NAME
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
echo JENKINS_TOOLS=${JENKINS_TOOLS}
echo WORKSPACE=${WORKSPACE}
echo JOB_NAME=${JOB_NAME}
echo GIT_REPOSITORY=${GIT_REPOSITORY}
echo GIT_BRANCH=${GIT_BRANCH}
echo CLEAN_ALL=${CLEAN_ALL}
echo CLEAN_DATA=${CLEAN_DATA}
echo CLEAN_BUILD=${CLEAN_BUILD}
echo RUN_TEST=${RUN_TEST}
echo RUN_REPORT=${RUN_REPORT}
echo RUN_PAPER=${RUN_PAPER}
echo NODE_NAME=$NODE_NAME

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


if [ ! -n "${JENKINS_TOOLS+1}" ]; then
  echo "JENKINS_TOOLS is not defined !!"
  exit 1
fi

if [ ! -n "${WORKSPACE+1}" ]; then
  echo "WORKSPACE is not defined !!"
  exit 1
fi

if [ ! -n "${JOB_NAME+1}" ]; then
  echo "JOB_NAME is not defined !!"
  exit 1
fi

if [ ! -n "${RUN_TEST+1}" ]; then
  echo "RUN_TEST is not defined !!"
  exit 1
fi

if [ ! -n "${RUN_REPORT+1}" ]; then
  echo "RUN_REPORT is not defined !!"
  exit 1
fi

if [ ! -n "${RUN_PAPER+1}" ]; then
  echo "RUN_PAPER is not defined !!"
  exit 1
fi

if [ ! -n "${NODE_NAME+1}" ]; then
  echo "NODE_NAME is not defined !!"
  exit 1
fi

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
#############   NODE SPECIFIC VALUES
#######################################################################################

# export DISPLAY=:0
#if [ "${NODE_NAME}" == "Cerberus" ]; then
#    export DISPLAY=:0
#fi




#######################################################################################
#############   Run the tests
#######################################################################################

if [ "${RUN_TEST}" == "true" ]; then
    mkdir -p ${WORKSPACE}/log/
    rm -f ${WORKSPACE}/log/*
    cd ${WORKSPACE}
    PYTHONPATH=${JENKINS_TOOLS} python ${JENKINS_TOOLS}/test_script.py  -o ${WORKSPACE}/${JOB_NAME}.xml -l ${WORKSPACE}/log/ -s ${WORKSPACE}/repository/
else
    touch ${WORKSPACE}/${JOB_NAME}.xml
fi

#######################################################################################
#############   Generate the reports
#######################################################################################

ICRA_PAPER_FOLDER=${WORKSPACE}/repository/icra2018_results/


if [ "${RUN_REPORT}" == "true" ]; then
    PYTHONPATH=${ICRA_PAPER_FOLDER} ${ICRA_PAPER_FOLDER}/violins.py ${WORKSPACE}/log/*.log --plot    ${WORKSPACE}/violins.pdf
    PYTHONPATH=${ICRA_PAPER_FOLDER} ${ICRA_PAPER_FOLDER}/violins.py ${WORKSPACE}/log/*.log --latex > ${WORKSPACE}/report.tex
    pdflatex ${WORKSPACE}/report.tex
fi

#######################################################################################
#############   Run the ICRA18 paper script
#######################################################################################



if [ "${RUN_PAPER}" == "true" ]; then
    mkdir -p ${ICRA_PAPER_FOLDER}/${NODE_NAME}
    rm ${ICRA_PAPER_FOLDER}/${NODE_NAME}/*
    cd ${ICRA_PAPER_FOLDER}
    paper_run.sh  ${ICRA_PAPER_FOLDER}/${NODE_NAME}
    make -C ${ICRA_PAPER_FOLDER}  ${NODE_NAME}_report.pdf
fi

cd ${WORKSPACE}
