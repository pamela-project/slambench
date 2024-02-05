#!/bin/bash

EXP_NUM=01
KITTI_PATH=./datasets/KITTI
NewerCollege_PATH=./datasets/NewerCollege
NSH_PATH=./datasets/NSH

if [ -z "$1" ]; then
    echo "No algorithm provided. Please provide an algorithm as an argument."
    exit 1
fi

case "$1" in
    "kitti")
        # ========================= KITTI 07 ===============================
        ./build/bin/slambench \
        -i ${KITTI_PATH}/2011_09_30_drive_0027/2011_09_30_drive_0027_sync.slam \
        -load /deps/aloam/lib/libaloam-original-library.so \
        -o ./results/log_files/overhead/A-LOAM_KITTI07_$EXP_NUM.txt \

        ./build/bin/slambench \
        -i ${KITTI_PATH}/2011_09_30_drive_0027/2011_09_30_drive_0027_sync.slam \
        -load /deps/legoloam/lib/liblegoloam-original-library.so \
        -o ./results/log_files/overhead/LeGO-LOAM_KITTI07_$EXP_NUM.txt \
        -configs /deps/legoloam/configs/HDL-64E.yaml
        ;;
    "newercollege")
        # ========================= NewerCollege ===============================
        ./build/bin/slambench \
        -i ${NewerCollege_PATH}/newer_college_short_quad_mid.slam \
        -load /deps/aloam/lib/libaloam-original-library.so \
        -o ./results/log_files/overhead/A-LOAM_NewerCollegeQM_$EXP_NUM.txt \

        ./build/bin/slambench \
        -i ${NewerCollege_PATH}/newer_college_short_quad_mid.slam \
        -load /deps/legoloam/lib/liblegoloam-original-library.so \
        -o ./results/log_files/overhead/LeGO-LOAM_NewerCollegeQM_$EXP_NUM.txt \
        -configs /deps/legoloam/configs/OS1-64.yaml
        ;;
    "nsh")
        # ========================= NSH ===============================
        ./build/bin/slambench \
        -i ${NSH_PATH}/nsh_indoor_and_outdoor.slam \
        -load /deps/aloam/lib/libaloam-original-library.so \
        -o ./results/log_files/overhead/A-LOAM_NSH_$EXP_NUM.txt \

        ./build/bin/slambench \
        -i ${NSH_PATH}/nsh_indoor_and_outdoor.slam \
        -load /deps/legoloam/lib/liblegoloam-original-library.so \
        -o ./results/log_files/overhead/LeGO-LOAM_NSH_$EXP_NUM.txt \
        -configs /deps/legoloam/configs/VLP-16.yaml
        ;;
    "all")
        # ========================= KITTI 07 ===============================
        ./build/bin/slambench \
        -i ${KITTI_PATH}/2011_09_30_drive_0027/2011_09_30_drive_0027_sync.slam \
        -load /deps/aloam/lib/libaloam-original-library.so \
        -o ./results/log_files/overhead/A-LOAM_KITTI07_$EXP_NUM.txt \

        ./build/bin/slambench \
        -i ${KITTI_PATH}/2011_09_30_drive_0027/2011_09_30_drive_0027_sync.slam \
        -load /deps/legoloam/lib/liblegoloam-original-library.so \
        -o ./results/log_files/overhead/LeGO-LOAM_KITTI07_$EXP_NUM.txt \
        -configs /deps/legoloam/configs/HDL-64E.yaml

        python3 ./scripts/visualize.py \
        -a ./results/log_files/overhead/A-LOAM_KITTI07_$EXP_NUM.txt ./results/log_files/overhead/LeGO-LOAM_KITTI07_$EXP_NUM.txt \
        -o ./results/experiments/overhead/KITTI07/
        
        # ========================= NewerCollege ===============================
        ./build/bin/slambench \
        -i ${NewerCollege_PATH}/newer_college_short_quad_mid.slam \
        -load /deps/aloam/lib/libaloam-original-library.so \
        -o ./results/log_files/overhead/A-LOAM_NewerCollegeQM_$EXP_NUM.txt \

        ./build/bin/slambench \
        -i ${NewerCollege_PATH}/newer_college_short_quad_mid.slam \
        -load /deps/legoloam/lib/liblegoloam-original-library.so \
        -o ./results/log_files/overhead/LeGO-LOAM_NewerCollegeQM_$EXP_NUM.txt \
        -configs /deps/legoloam/configs/OS1-64.yaml

        python3 ./scripts/visualize.py \
        -a ./results/log_files/overhead/A-LOAM_NewerCollegeQM_$EXP_NUM.txt ./results/log_files/overhead/LeGO-LOAM_NewerCollegeQM_$EXP_NUM.txt \
        -o ./results/experiments/overhead/NewerCollegeQM/

        # ========================= NSH ===============================
        ./build/bin/slambench \
        -i ${NSH_PATH}/nsh_indoor_and_outdoor.slam \
        -load /deps/aloam/lib/libaloam-original-library.so \
        -o ./results/log_files/overhead/A-LOAM_NSH_$EXP_NUM.txt \

        ./build/bin/slambench \
        -i ${NSH_PATH}/nsh_indoor_and_outdoor.slam \
        -load /deps/legoloam/lib/liblegoloam-original-library.so \
        -o ./results/log_files/overhead/LeGO-LOAM_NSH_$EXP_NUM.txt \
        -configs /deps/legoloam/configs/VLP-16.yaml

        python3 ./scripts/visualize.py \
        -a ./results/log_files/overhead/A-LOAM_NSH_$EXP_NUM.txt ./results/log_files/overhead/LeGO-LOAM_NSH_$EXP_NUM.txt \
        -o ./results/experiments/overhead/NSH/
        ;;
    *)
        echo "Invalid Input!"
        ;;
esac









