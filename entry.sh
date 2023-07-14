#!/bin/bash

case "$1" in
    --test)
        make g2o
        make opencv
        make suiteparse
        make lsdslam
        make slambench APPS=lsdslam
        make datasets/ICL_NUIM/living_room_traj2_loop.slam
        ./build/bin/slambench -i datasets/ICL_NUIM/living_room_traj2_loop.slam -load ./build/lib/liblsdslam-cpp-library.so
        ;;
    --dataset)
        if [ -z "$2" ]; then
            echo "Missing dataset name."
            exit 1
        fi
        make "$2"
        ;;
    --build-gui)
        if [ -z "$2" ] || [ -z "$3" ]; then
            echo "Missing data path or algorithm path."
            exit 1
        fi
        ARG DEBIAN_FRONTEND=noninteractive
        apt install -y vainfo mesa-va-drivers
        ENV LIBVA_DRIVER_NAME=d3d12
        ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib
        slam_file=$(find "$dataset_folder" -type f -name "*.slam")
        # Extract the path to the .slam file
        slam_file_path=$(dirname "$slam_file")
        ./build/bin/slambench -i "$slam_file_path" -load "$3" --gui true
        ;;
    --build-cli)
        if [ -z "$2" ] || [ -z "$3" ]; then
            echo "Missing data path or algorithm path."
            exit 1
        fi
        slam_file=$(find "$dataset_folder" -type f -name "*.slam")
        # Extract the path to the .slam file
        slam_file_path=$(dirname "$slam_file")
        ./build/bin/slambench -i "$slam_file_path" -load "$3"
        ;;
    *)
        echo "Invalid argument. Usage: $0 [--test | --dataset <dataset_name> | --build <data_path> <algorithm_path>]"
        exit 1
        ;;
esac
