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
    --bench-gui)
        if [ -z "$2" ] || [ -z "$3" ]; then
            echo "Missing data path or algorithm path."
            exit 1
        fi
        ARG DEBIAN_FRONTEND=noninteractive
        apt install -y vainfo mesa-va-drivers
        ENV LIBVA_DRIVER_NAME=d3d12
        ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib
        additional_args="${@:4}"
        # set path for slam
        slam_file=$(find "./datasets" -type f -name "$2")
        echo "$slam_file"
        if [ -f "$3" ]; then
            ./build/bin/slambench -i "$slam_file" -load "$3" --gui true $additional_args
        else
            vol_name=$(basename "$(dirname "$3")")
            cd "/deps/$vol_name/"
            cmake .
            make 
            /slambench/build/bin/slambench -i "$slam_file" -load "$3" --gui true $additional_args
        fi
        ;;
    --bench-cli)
        if [ -z "$2" ] || [ -z "$3" ]; then
            echo "Missing data path or algorithm path."
            exit 1
        fi
        additional_args="${@:4}"
        slam_file=$(find ./datasets -type f -name "$2")
        echo "$slam_file"
        if [ -f "$3" ]; then
            ./build/bin/slambench -i "$slam_file" -load "$3" $additional_args
        else
            vol_name=$(basename "$(dirname "$3")")
            cd "/deps/$vol_name/"
            cmake .
            make 
            /slambench/build/bin/slambench -i "$slam_file" -load "$3" $additional_args
        fi
        ;;
    --interactive)
        /bin/bash
        ;;
    --list_datasets)
        make datasets
        ;;
    *)
        echo "Invalid argument. Usage: $0 [--test | --dataset <dataset_name> | --build <data_path> <algorithm_path>]"
        exit 1
        ;;
        
esac
