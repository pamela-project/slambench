How to run the build script:

CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda CLEAN_BUILD=false CLEAN_DATA=false CLEAN_ALL=false GIT_BRANCH=bruno-dev GIT_REPOSITORY="https://github.com/pamela-project/slambench2.git" WORKSPACE=~/workspace ./docker/build.sh

How to run the test script:

JOB_NAME=slambench2 NODE_NAME=`hostname` RUN_PAPER=false RUN_REPORT=true RUN_TEST=true WORKSPACE=~/workspace JENKINS_TOOLS=`pwd`/docker/ `pwd`/docker/test.sh 