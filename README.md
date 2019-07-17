# README #

SLAMBench 2.0

  Copyright (c) 2014-2018 University of Edinburgh, Imperial College, University of Manchester.
  Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

[![Build Status](https://travis-ci.org/pamela-project/slambench2.svg?branch=master)](https://travis-ci.org/pamela-project/slambench2)

## Most frequent questions ##

### Where are the algorithms ? ###

If SLAMBench is already compatible with a wide range of algorithms they are not included in this repository. 
However you can easily integrate those algorithms using the command :

```
make usecases
```

This command will explain in details how to integrate algorithms that are already compatible with slambench.

## What is slambench ##

SLAMBench is a SLAM performance benchmark that combines a framework for quantifying quality-of-result with instrumentation of accuracy, execution time, memory usage and energy consumption. It also include a graphical interface to visualize these information.

SLAMBench offers a platform for a broad spectrum of future research in jointly exploring the design space of algorithmic and implementation-level optimisations. It targets desktop, laptop, mobile and embedded platforms. Some of the benchmarks (in particular KFusion) were tested on Ubuntu, OS X and Android (more information about android here [https://github.com/bbodin/slambench-android](https://github.com/bbodin/slambench-android)).

SLAMBench has currently been used with the following SLAM algorithms :

 * ElasticFusion [Whelan et al, IJRR'16]: CUDA as distributed by https://github.com/mp3guy
 * InfiniTAMv2 [Kahler et al, ISMAR'15]: C++, OpenMP and CUDA versions as distributed by https://github.com/victorprad/
 * KinectFusion [Newcombe et al. ISMAR'11]: C++, OpenMP, OpenCL and CUDA inspired by https://github.com/GerhardR
 * LSDSLAM [Engel et al, ECCV'14]: C++, and threaded as distributed by https://github.com/tum-vision/ and modified by https://github.com/mp3guy
 * MonoSLAM [Davison et al, TPAMI'07]: Original version as distributed by https://github.com/hanmekim/
 * OKVIS [Leutenegger et al, IJRR'15]: Original version as distributed by https://github.com/ethz-asl
 * PTAM [Klein et al, ISMAR'07 and ECCV'08]: Original version as distributed by https://github.com/Oxford-PTAM/
 * SVO [Forster et al, ICRA'14]: Original version as distributed by https://github.com/uzh-rpg/rpg_svo/ (a more recent version available at http://rpg.ifi.uzh.ch/svo2.html)

  **IMPORTANT: If you use any of those algorithms in scientific publications, you should refer to the respective publications.**


In addition, if you use SLAMBench in scientific publications, we would appreciate citations to the following paper [https://www.research.ed.ac.uk/](https://www.research.ed.ac.uk/portal/en/publications/slambench2-multiobjective-headtohead-benchmarking-for-visual-slam(c3d189db-68ae-4182-8e46-d812db2955c9).html):
```
@inproceedings{Bodin2018,
	author    = "Bruno Bodin and Harry Wagstaff and Sajad Saeedi and Luigi Nardi and Emanuele Vespa and Mayer, {John H} and Andy Nisbet and Mikel Luján and Steve Furber and Davison, {Andrew J} and Kelly, {Paul H.J.} and Michael O'Boyle",
	title     = "SLAMBench2: Multi-Objective Head-to-Head Benchmarking for Visual SLAM",
  	booktitle = "{IEEE Intl. Conf. on Robotics and Automation (ICRA)}",
   	year = {2018},
   	month = {May}
    }
```

# Table of content #

This README includes the following sections:

 * How to set up SLAMBench ?
 * How to run a benchmark with SLAMBench ?
 * How to add a new benchmark in SLAMBench ?


## How to set up SLAMBench ? ##

As SLAMBench deals with multiple SLAM algorithms, dependencies might be difficult to install on any systems.
To ease the usage of SLAMbench we provide auto-installation of dependencies and we recommend to use fresh installation of the operating systems Ubuntu 14/16 or Fedora 24/25/26/27 that are known to work fine.

### Dependencies installations ###

#### Required by SLAMBench framework ####
* CMake 2.8.11 or higher is required.
* Make
* GCC C/C++
* Boost (Optionnal)
* GLUT (Optionnal)

#### Required by benchmarks dependencies ####
* Git
* Mercurial
* wget
* unzip
* lapack
* blas
* findutils
* cvs
* glog
* gflags

#### To install them ####

With Fedora 29:
```dnf install -y yaml-cpp-devel gtk2-devel mesa-libEGL-devel vtk-devel cmake make git mercurial wget unzip gcc gcc-c++ lapack blas lapack-devel blas-devel findutils  cvs  glut-devel glew-devel boost-devel glog-devel gflags-devel libXmu-devel```

With Fedora 24: 
```dnf install -y yaml-cpp-devel gtk2-devel vtk-devel cmake make git mercurial wget unzip gcc gcc-c++ lapack blas lapack-devel blas-devel findutils  cvs  glut-devel glew-devel boost-devel glog-devel gflags-devel libXmu-devel```

With Ubuntu 16.10: 
``` apt-get -y install libvtk6.3 libvtk6-dev unzip libflann-dev wget mercurial git gcc cmake python-numpy freeglut3 freeglut3-dev libglew1.5 libglew1.5-dev libglu1-mesa libglu1-mesa-dev libgl1-mesa-glx libgl1-mesa-dev libxmu-dev libxi-dev  libboost-all-dev cvs libgoogle-glog-dev libatlas-base-dev gfortran  gtk2.0 libgtk2.0-dev  libyaml-dev build-essential bison flex libyaml-cpp-dev ```

With Ubuntu 16.04: 
``` apt-get -y install libvtk6.2 libvtk6-dev unzip libflann-dev wget mercurial git gcc cmake python-numpy freeglut3 freeglut3-dev libglew1.5 libglew1.5-dev libglu1-mesa libglu1-mesa-dev libgl1-mesa-glx libgl1-mesa-dev libxmu-dev libxi-dev  libboost-all-dev cvs libgoogle-glog-dev libatlas-base-dev gfortran  gtk2.0 libgtk2.0-dev libproj9 libproj-dev libyaml-0-2 libyaml-dev libyaml-cpp-dev libhdf5-dev libhdf5-dev```

With Ubuntu 14.04: 
``` apt-get -y install libvtk6-dev unzip libflann-dev wget mercurial git gcc cmake python-numpy freeglut3 freeglut3-dev libglew1.5 libglew1.5-dev libglu1-mesa libglu1-mesa-dev libgl1-mesa-glx libgl1-mesa-dev libxmu-dev libxi-dev  libboost-all-dev cvs libgoogle-glog-dev libatlas-base-dev gfortran  gtk2.0 libgtk2.0-dev ```

#### Special requirements for CUDA ####

Tu run the CUDA implementation of some of the algorithms, you will need extra dependencies.

With Ubuntu: 
``` apt-get -y install  nvidia-cuda-toolkit clinfo``` 

With Fedora: 
``` yum install cuda ``` 

### Compilation of SLAMBench and its benchmarks ###

#### 1. dependencies ####

Install dependencies first [NOTE can be installed by the user on its system as well]:

```
make deps
```
The idea is to maximise the chance of a good build, by selection the best cocktail of libraries.
This will downdload and compile the following applications : Brisk, Ceres, CVD, Eigen3, Flann, FreeImage, G2O, Gvars, OpenCV, OpenGV, OpenTuner, Pangolin, PCL, Suitesparse, TooN.

more informations are available in the ``framework/makefiles/README.md`` file.

#### 2. SLAMBench framework ####

SLAMBench is a framework that can be compiled by simply running:

```
make slambench
```

Although, by doing this, you only compile the libraries of SLAMBench.

#### 3. Usescases ####

To compile a specific use-case, you will need to specify identifiers :

```
make slambench APPS=kfusion,lsdslam
```

The current benchmarks identifiers are efusion, infinitam, kfusion, lsdslam, monoslam, okvis, ptam, orbslam2, svo.
You will find more information to downdload and compile use-cases with the ``make usecases`` command.

#### 4. Datasets ####

To test a SLAM algorithm you can use a Live camera, or a dataset. 
SLAMBench provides tools to automatically download some of the most popular datasets, that is ICL-NUIM and TUM RGB-D. 
The file format (*.slam) will then include all the most important information about the dataset, those are **Camera calibration setting**, **initial position of the sensors**, and the **ground truth**.

As an example to download and generate the Living Room Trajectory 2 from the ICLNUIM dataset, you can run the following :

```
> make datasets/ICL_NUIM/living_room_traj2_loop.slam
```

A complete list of the datasets available is provided by the command ``make datasetslist``.


## How to run an existing algorithm with SLAMBench ? ##

Once you have compile a benchmark, there are several ways to run it.
For each implementation of this benchmark, you will find a specific library. 
As an example, with KinectFusion, after running ```make benchmark APPS=kfusion```, you may found the following libraries in the ```build/lib``` directory :

```
> ls build/lib/libkfusion-*-library.so

build/lib/libkfusion-cpp-library.so   
build/lib/libkfusion-notoon-library.so      
build/lib/libkfusion-openmp-library.so
build/lib/libkfusion-cuda-library.so  
build/lib/libkfusion-opencl-library.so

```

We can see five different implementations (cpp, notoon, and openmp, cuda and opencl). The list of available binaries depends of the dependencies you installed beforehand. For example, you need CUDA to compile the kfusion-cuda. A complete list of the dependencies is available at the end of this README.



### Running a benchmark (e.g. KinectFusion) ###

To run one algorithm you will need to use a **loader**. 
There is currently two different loaders supported, **benchmark** and **pangolin**.
Both loader are used the same way, except that **benchmark** is a command line application dedicated to measurements, while **pangolin** is a graphical user interface less precise in term of measurement but which provide a good interface for demonstrations.


Each loader has a series of parameters to specify such as the dataset location, or the libraries to run. 
The list of those parameters is available by using the "--help" parameters.

```
> ./build/bin/benchmark_loader --help 
 == SLAMBench Configuration ==
  Available parameters :
   -fl            --frame-limit           : last frame to compute (Default=0)
   -o             --log-file              : Output log file (Default=)
   -i             --input                 : Specify the input file or mode. (Default=)
   -load          --load-library          : Load a specific SLAM library. (Default=)
   -dse           --dse                   : Output solution space of parameters. (Default=false)
   -h             --help                  : Print the help. (Default=false)
   -nf            --negative-focal-length : negative focal length (Default=false)
   -realtime      --realtime-mode         : realtime frame loading mode (Default=false)
   -realtime-mult --realtime-multiplier   : realtime frame loading mode (Default=1)
   -fo            --file-output           : File to write slamfile containing outputs (Default=)
    
```


Then if you run the loader again, while providing a dataset file ```-i dataset.slam```, you will see new parameters dedicated to the dataset : 


```
> ./build/bin/benchmark_loader -i datasets/ICL_NUIM/living_room_traj2_loop.slam --help
 == SLAMBench Configuration ==
  Available parameters :
   ....
   -Camera-intrisics --Camera-intrisics       : (Default=nullptr  Current=0.751875,1,0.4992185,0.4989583)
   -Depth-intrisics  --Depth-intrisics        : (Default=nullptr  Current=0.751875,1,0.4992185,0.4989583)
   -Depth-dip        --Depth-disparity-params : (Default=nullptr  Current=0.001,0)
   -Camera-intrisics --Camera-intrisics       : (Default=nullptr  Current=0.751875,1,0.4992185,0.4989583)


```

Finally is you add a library name ```-load libname```, more parameter can be seen : 

```
> ./build/bin/benchmark_loader -i datasets/ICL_NUIM/living_room_traj2_loop.slam -load ./build/lib/libkfusion-cpp-library.so  --help
 == SLAMBench Configuration ==
  Available parameters :

    ....

   -c                --compute-size-ratio     : Compute ratio (Default=1)
   -r                --integration-rate       : integration-rate  (Default=2)
   -t                --tracking-rate          : tracking-rate     (Default=1)
   -z                --rendering-rate         : rendering-rate    (Default=4)
   -l                --icp-threshold          : icp-threshold     (Default=1e-05)
   -m                --mu                     : mu                (Default=0.1)
   -s                --volume-size            : volume-size       (Default=8,8,8)
   -d                --volume-direction       : volume-direction  (Default=4,4,4)
   -v                --volume-resolution      : volume-resolution (Default=256,256,256)
   -y1               --pyramid-level1         : pyramid-level1    (Default=10)
   -y2               --pyramid-level2         : pyramid-level2    (Default=5)
   -y3               --pyramid-level3         : pyramid-level3    (Default=4)

```


You can run a loader with **only one dataset** at a time and **it must be specified first**.

In the next section we will explain how to use SLAMBench to evaluate the performance of a SLAM algorithm.

### Evaluating a benchmark (eg. KinectFusion) ###

SLAMBench works with Metrics and Outputs elements. 
When you run the ```benchmark_loader``` or the ```pangolin_loader``` these are those elements that you can visualize.
Metrics are components generated by SLAMbench framework really, while Outputs are generated by the algorithm or may be elements post-processed by SLAMbench (such as the aligned trajectory with the ground truth).

Les us run the benchmark loader. Its output is composed of two main parts, the ```Properties``` section, and the ```Statistics``` section. 
the properties section details all the parameters used for the experiment (could been changed or not via the command line). 
the statistics section repart all the outputs and metrics selection for output in the benchmark loader.

```
> ./build/bin/benchmark_loader -i datasets/ICL_NUIM/living_room_traj2_loop.slam -load ./build/lib/libkfusion-cpp-library.so 

SLAMBench Report run started:	2018-02-02 04:41:31

Properties:
=================

frame-limit: 0
log-file: 
input: datasets/ICL_NUIM/living_room_traj2_loop.slam
load-library: ./build/lib/libkfusion-cpp-library.so
dse: false
help: false
negative-focal-length: false
realtime-mode: false
realtime-multiplier: 1
file-output: 
Camera-intrisics: 0.751875,1,0.4992185,0.4989583
Depth-intrisics: 0.751875,1,0.4992185,0.4989583
Depth-disparity-params: 0.001,0
Camera-intrisics: 0.751875,1,0.4992185,0.4989583
compute-size-ratio: 1
integration-rate: 2
tracking-rate: 1
rendering-rate: 4
icp-threshold: 1e-05
mu: 0.1
volume-size: 8,8,8
volume-direction: 4,4,4
volume-resolution: 256,256,256
pyramid-level1: 10
pyramid-level2: 5
pyramid-level3: 4
Statistics:
=================

Frame Number	Timestamp	Duration_Frame	GPU_Memory	CPU_Memory		Duration_Preprocessing	Duration_Tracking	Duration_Integration	Duration_Raycasting	Duration_Render	X	Y	ZATE_Frame
1	0.0000000000	0.7679200000	0	623801799		0.1254800000	0.0195420000	0.0561620000	0.0000030000	0.5667170000	4.0000000000	4.0000000000	4.0000000000	0.0000002980
2	1.0000000000	0.2003970000	0	623801799		0.1242030000	0.0156470000	0.0581670000	0.0000000000	0.0023710000	4.0000000000	4.0000000000	4.0000000000	0.0010031639
3	2.0000000000	0.1989980000	0	623801799		0.1233680000	0.0152360000	0.0580180000	0.0000000000	0.0023690000	4.0000000000	4.0000000000	4.0000000000	0.0055015362
4	3.0000000000	0.7518580000	0	623801799		0.1220660000	0.0152080000	0.0563070000	0.5559520000	0.0023170000	4.0000000000	4.0000000000	4.0000000000	0.0036504765
5	4.0000000000	1.3683420000	0	623801799		0.1240890000	0.0767240000	0.0581630000	0.5504240000	0.5589330000	3.9957129955	4.0020360947	4.0009112358	0.0021276891
...
```




## How to add a new benchmark in SLAMBench ? ##

The main reason to provide a new version of SLAMBench is not only because of the introduction of new benchmarks but also because we provide now 
a clear and specific API for SLAM algorithms to be implemented in order to add a new algorithm.

```
void sb_new_slam_configuration(SLAMBenchConfiguration ** slam_settings) ;
bool sb_init_slam_system(SLAMBenchConfiguration * slam_settings) ;
bool sb_update_frame (Sensor * type) ;
bool sb_process_once () ;
bool sb_clean_slam_system();
bool sb_update_outputs(SLAMBenchUI * );
```

**If each of those functions are correctly implemented for a specific implementation of a specific algorithm, then this algorithm is compatible with SLAMBench and ben be evaluated as well.**

In this section we will present those functions one by one.


### bool sb_new_slam_configuration(SLAMBenchLibraryHelper * slam_settings)  ###

This function is called first, and only once, SLAM systems is expected to provide its parameters.

Example :

```

 bool sb_new_slam_configuration(SLAMBenchLibraryHelper * slam_settings)  {
     slam_settings->addParameter(TypedParameter<float>("c", "confidence",     "Confidence",      &confidence, &default_confidence));
     slam_settings->addParameter(TypedParameter<float>("d", "depth",          "Depth",           &depth,      &default_depth));
     slam_settings->addParameter(TypedParameter<int>  ("td", "textureDim",      "textureDim",       &textureDim,      &default_textureDim));
     return true;
 }

```

should always return ``true`` or an exception will be raised.

 
### bool sb_init_slam_system(SLAMBenchLibraryHelper * slam_settings)   ###

This function is called second, and only once, SLAM systems is expected to allocate memory, retrieve sensor informations.


To retrieve sensor there is ``SensorFinder`` :
```
	slambench::io::CameraSensorFinder sensor_finder;
	auto rgb_sensor = sensor_finder.FindOne(slam_settings->get_sensors(), {{"camera_type", "rgb"}});
```

SLAM systems are also expected to define there output, there is one mandatory output, the pose:

```
pose_output = new slambench::outputs::Output("Pose", slambench::values::VT_POSE, true);
slam_settings->GetOutputManager().RegisterOutput(pose_output);
should always return ``true`` or an exception will be raised.
```

### bool sb_update_frame (SLAMBenchLibraryHelper *  slam_settings, slambench::io::SLAMFrame* s)  ### 

The frame ```s``` is the next frame to process in the dataset (ordered by timestamps).

When ```sb_update_frame``` return ```false```, then ```sb_update_frame``` will be directly call again with the next frame, if it return ```true````, then ```sb_process_once``` will be called once.

### bool sb_process_once (SLAMBenchLibraryHelper * slam_settings)   ###  
 
should always return ``true`` or an exception will be raised.

### bool sb_clean_slam_system()  ###

This function is called last, and only once, SLAM systems is expected to clean everything (free memory).

```
 bool sb_clean_slam_system() {
     delete eFusion;
     delete inputRGB;
     delete inputDepth;
     return true;
 }

```

should always return ``true`` or an exception will be raised.

### bool sb_update_outputs(SLAMBenchLibraryHelper * slam_settings, const slambench::TimeStamp *timestamp)  ###  

The algorithm will return visible outputs (Pose, Point cloud, Frames) as defined by the ```sb_init_slam_system``` function.

Example :

```

 bool sb_update_outputs(SLAMBenchLibraryHelper *lib, const slambench::TimeStamp *ts_p) {
		slambench::TimeStamp ts = *ts_p;

		if(pose_output->IsActive()) {
			// Get the current pose as an eigen matrix
			Eigen::Matrix4f mat = eFusion->getCurrPose();

			std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
			pose_output->AddPoint(ts, new slambench::values::PoseValue(mat));
		}


```

should always return ``true`` or an exception will be raised.

## About python scripts ##

When running DSE, we generate lots of log files, generally inside a output directory.
There is a script to preprocess and compress these logs :

./framework/tools/python/readlog.py ./output/lv3_efcuda/ -s lv3_efcuda.gkl



# Know issues #

### Know issue with CUDA ###

Applications using CUDA require GCC 4.9 to work. To specify a new gcc compiler for CUDA only, you can use the ```CUDA_HOST_COMPILER``` flag as follows :
```
make benchmark APPS=kfusion CUDA_HOST_COMPILER=/usr/local/gcc-4.9/bin/c++
```

Modern O.S. are now using more recent version of this compiler, this introduce several compatibility issues.
To fix one of them, in the compilation process, when compiling CUDA application we use the ``` -D_GLIBCXX_USE_CXX11_ABI=0 ``` flag.
# TODO List #

* Add office room dataset

# Release history #

Version 2.1 (Jul 2019)

 * Add OSX support for benchmark_loader with kfusion (no memory profiling yet).

Version 2.0 (Feb 2018)

 * This release is a complete new version

Release candidat 1.1 (17 Mar 2015)

  * Bugfix : Move bilateralFilterKernel from preprocessing to tracking
  * Bugfix : Wrong interpretation of ICP Threshold parameter.
  * Esthetic : Uniformisation of HalfSampleRobustImage kernel
  * Performance : Change float3 to float4 for the rendering kernels (No effect on OpenCL, but high performance improvement with CUDA)
  * Performance : Add a dedicated buffer for the OpenCL rendering
  * Feature : Add OSX support

Release candidat 1.0 (12 Nov 2014)

  * First public release

