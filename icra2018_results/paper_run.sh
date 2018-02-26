#!/bin/bash

set -x

if [ -z ${1+x} ]; then
    echo "This command requires one argument, the directory to store log files";
    exit 1;
fi

if [ -z ${2+x} ]; then
    echo "Execute the full benchmark with no extra arguments." ;
    DEBUG_ARGUMENTS="";
else
    echo "Execute the debug mode with extra arguments '$2'.";
    DEBUG_ARGUMENTS=$2;
fi

LOG_DIRECTORY=$1

if [ -z ${DISPLAY+x} ]; then echo "DISPLAY is not set"; else echo "DISPLAY is set to '$DISPLAY'"; fi

datasets=(
    "datasets/ICL_NUIM/living_room_traj0_loop.slam"
    "datasets/ICL_NUIM/living_room_traj1_loop.slam"
    "datasets/ICL_NUIM/living_room_traj2_loop.slam"
    "datasets/ICL_NUIM/living_room_traj3_loop.slam"
    "datasets/TUM/freiburg1/rgbd_dataset_freiburg1_rpy.slam"
    "datasets/TUM/freiburg1/rgbd_dataset_freiburg1_xyz.slam"
    "datasets/TUM/freiburg2/rgbd_dataset_freiburg2_rpy.slam"
    "datasets/TUM/freiburg2/rgbd_dataset_freiburg2_xyz.slam"
)

run_prepare () {
    
    if [ -z ${SKIP_CLEAN+x} ]; then
	make clean
	make cleandatasets	
    else
	echo "WARNING !! We will skip the cleaning step.";
    fi
    make slambench
    make benchmark APPS=kfusion,efusion,orbslam2,lsdslam,infinitam
    make ./benchmarks/orbslam2/src/original/Vocabulary/ORBvoc.txt
    
    for i in "${datasets[@]}"
    do
	make $i
    done
	

}

run_memory () {

    if [ -z ${1+x} ]; then echo "run_memory requires one argument."; exit 1 ; fi

    dataset=$1
    dataset_name=`basename -s .slam ${dataset}`
    
    if [ ! -f ${dataset}    ]; then continue          ; fi
    
	      
    
    ./build/bin/benchmark_loader  -i ${dataset}  -load  build/lib/liblsdslam-cpp-library.so -o ${LOG_DIRECTORY}/memory_lsdslam_${dataset_name}.log ${DEBUG_ARGUMENTS}  || exit 1
    ./build/bin/benchmark_loader  -i ${dataset}  -load  build/lib/libkfusion-cpp-library.so   -o ${LOG_DIRECTORY}/memory_kfusion_${dataset_name}.log ${DEBUG_ARGUMENTS}  || exit 1
    ./build/bin/benchmark_loader  -i ${dataset}  -load  build/lib/libinfinitam-cpp-library.so   -o ${LOG_DIRECTORY}/memory_infinitam_${dataset_name}.log ${DEBUG_ARGUMENTS}  || exit 1
    ./build/bin/benchmark_loader  -i ${dataset}  -load  build/lib/liborbslam2-original-library.so -voc ./benchmarks/orbslam2/src/original/Vocabulary/ORBvoc.txt -o ${LOG_DIRECTORY}/memory_orbslam2_${dataset_name}.log  ${DEBUG_ARGUMENTS}  || exit 1
   
	
	if [  -f  build/lib/libkfusion-cuda-library.so   ];
	then        
	    ./build/bin/benchmark_loader  -i ${dataset}  -load  build/lib/libkfusion-cuda-library.so   -o ${LOG_DIRECTORY}/memory_kfusion_${dataset_name}.log ${DEBUG_ARGUMENTS}  || exit 1
	fi
	if [  -f  build/lib/libefusion-cuda-library.so   ];
	then   
	    ./build/bin/benchmark_loader  -i ${dataset}  -load  build/lib/libefusion-cuda-library.so  --textureDim 512 --nodeTextureDim 2048 -o ${LOG_DIRECTORY}/memory_efusion_${dataset_name}.log  ${DEBUG_ARGUMENTS}  || exit 1
	fi
	if [  -f  build/lib/libinfinitam-cuda-library.so   ];
	then     
	    ./build/bin/benchmark_loader  -i ${dataset}  -load  build/lib/libinfinitam-cuda-library.so   -o ${LOG_DIRECTORY}/memory_infinitam_${dataset_name}.log  ${DEBUG_ARGUMENTS} || exit 1
	fi

    
}


run_violins () {

    if [ -z ${2+x} ]; then echo "run_violins requires two arguments."; exit 1 ; fi
    
    for i in "${datasets[@]}"
    do
	dataset=$i
	dataset_name=`basename -s .slam ${dataset}`
	library=$1
	library_name=`basename -s -library.so ${library}`
	arguments=$2
	output_log=${LOG_DIRECTORY}/violons_${library_name}_${dataset_name}.log
	if [ ! -f ${library}    ]; then continue          ; fi 
	if [ ! -f ${dataset}    ]; then continue          ; fi 
	if [   -f ${output_log} ]; then rm  ${output_log} ; fi 
	./build/bin/benchmark_loader  -i ${dataset}  -load ${library} ${arguments} -o ${LOG_DIRECTORY}/violons_${library_name}_${dataset_name}.log    ${DEBUG_ARGUMENTS}  || exit 1
    done
}

run_prepare

run_memory datasets/ICL_NUIM/living_room_traj2_loop.slam 

run_violins "build/lib/liblsdslam-cpp-library.so" ""
run_violins "build/lib/liborbslam2-original-library.so" "-voc ./benchmarks/orbslam2/src/original/Vocabulary/ORBvoc.txt"
run_violins "build/lib/libkfusion-cpp-library.so" ""
run_violins "build/lib/libinfinitam-cpp-library.so" ""


run_violins "build/lib/libkfusion-cuda-library.so" ""
run_violins "build/lib/libefusion-cuda-library.so" "--textureDim 512 --nodeTextureDim 2048"
run_violins "build/lib/libinfinitam-cuda-library.so" ""

