#!/usr/bin/python
###
###  Please remember to have the jenkins_tools directory available in PYTHONPATH
###
###

import os
import sys
import argparse
import re
import commons.jenkins_test as jenkinstest
import platform
from random import shuffle
import copy

def printerr(str) :
    sys.stderr.write( str + "\n")

################# DEFINE FUNCTIONS  #######################

def generate_test_units(OUTPUT_LOG_DIRECTORY, SKIP, SLAMBENCH2_DIRECTORY) :

    testsuite_name = "SLAMBench"
    
    env_variables = {

    }


    datasets  = {

        "MONO,RGBD" : {
        
        
    "living_room_traj0_loop" : SLAMBENCH2_DIRECTORY + "/datasets/ICL_NUIM/living_room_traj0_loop.slam" ,
    "living_room_traj1_loop" : SLAMBENCH2_DIRECTORY + "/datasets/ICL_NUIM/living_room_traj1_loop.slam" ,
     "living_room_traj2_loop" : SLAMBENCH2_DIRECTORY + "/datasets/ICL_NUIM/living_room_traj2_loop.slam" ,
    "living_room_traj3_loop" : SLAMBENCH2_DIRECTORY + "/datasets/ICL_NUIM/living_room_traj3_loop.slam" ,
    
    "freiburg1_xyz"   : SLAMBENCH2_DIRECTORY + "/datasets/TUM/freiburg1/rgbd_dataset_freiburg1_xyz.slam" ,
    "freiburg1_rpy"   : SLAMBENCH2_DIRECTORY + "/datasets/TUM/freiburg1/rgbd_dataset_freiburg1_rpy.slam" ,
    "freiburg1_360"   : SLAMBENCH2_DIRECTORY + "/datasets/TUM/freiburg1/rgbd_dataset_freiburg1_360.slam" ,
    "freiburg1_floor" : SLAMBENCH2_DIRECTORY + "/datasets/TUM/freiburg1/rgbd_dataset_freiburg1_floor.slam" ,
    "freiburg1_desk"  : SLAMBENCH2_DIRECTORY + "/datasets/TUM/freiburg1/rgbd_dataset_freiburg1_desk.slam" ,
    "freiburg1_desk2" : SLAMBENCH2_DIRECTORY + "/datasets/TUM/freiburg1/rgbd_dataset_freiburg1_desk2.slam" ,
    "freiburg1_room"  : SLAMBENCH2_DIRECTORY + "/datasets/TUM/freiburg1/rgbd_dataset_freiburg1_room.slam" ,
    
    "freiburg2_xyz"   : SLAMBENCH2_DIRECTORY + "/datasets/TUM/freiburg2/rgbd_dataset_freiburg2_xyz.slam" ,
    "freiburg2_rpy"   : SLAMBENCH2_DIRECTORY + "/datasets/TUM/freiburg2/rgbd_dataset_freiburg2_rpy.slam" ,
    "freiburg2_360_hemisphere": SLAMBENCH2_DIRECTORY + "/datasets/TUM/freiburg2/rgbd_dataset_freiburg2_360_hemisphere.slam" ,
    "freiburg2_360_kidnap": SLAMBENCH2_DIRECTORY + "/datasets/TUM/freiburg2/rgbd_dataset_freiburg2_360_kidnap.slam" ,
    "freiburg2_desk": SLAMBENCH2_DIRECTORY + "/datasets/TUM/freiburg2/rgbd_dataset_freiburg2_desk.slam" ,
    "freiburg2_desk_with_person": SLAMBENCH2_DIRECTORY + "/datasets/TUM/freiburg2/rgbd_dataset_freiburg2_desk_with_person.slam" ,
    "freiburg2_large_no_loop": SLAMBENCH2_DIRECTORY + "/datasets/TUM/freiburg2/rgbd_dataset_freiburg2_large_no_loop.slam" ,
    "freiburg2_large_with_loop": SLAMBENCH2_DIRECTORY + "/datasets/TUM/freiburg2/rgbd_dataset_freiburg2_large_with_loop.slam" ,
    "freiburg2_pioneer_360": SLAMBENCH2_DIRECTORY + "/datasets/TUM/freiburg2/rgbd_dataset_freiburg2_pioneer_360.slam" ,
    "freiburg2_pioneer_slam": SLAMBENCH2_DIRECTORY + "/datasets/TUM/freiburg2/rgbd_dataset_freiburg2_pioneer_slam.slam" ,
    "freiburg2_pioneer_slam2": SLAMBENCH2_DIRECTORY + "/datasets/TUM/freiburg2/rgbd_dataset_freiburg2_pioneer_slam2.slam" ,
    "freiburg2_pioneer_slam3": SLAMBENCH2_DIRECTORY + "/datasets/TUM/freiburg2/rgbd_dataset_freiburg2_pioneer_slam3.slam" ,

            } ,

        "MONO,STEREO" : {
  #  
   "MH1": SLAMBENCH2_DIRECTORY + "/datasets/EuRoCMAV/machine_hall/MH_01_easy/MH_01_easy.slam" ,
   "MH2": SLAMBENCH2_DIRECTORY + "/datasets/EuRoCMAV/machine_hall/MH_02_easy/MH_02_easy.slam" ,
   "MH3": SLAMBENCH2_DIRECTORY + "/datasets/EuRoCMAV/machine_hall/MH_03_medium/MH_03_medium.slam" ,
   "MH4": SLAMBENCH2_DIRECTORY + "/datasets/EuRoCMAV/machine_hall/MH_04_difficult/MH_04_difficult.slam" ,
   "MH5": SLAMBENCH2_DIRECTORY + "/datasets/EuRoCMAV/machine_hall/MH_05_difficult/MH_05_difficult.slam" ,
    
   "VR11": SLAMBENCH2_DIRECTORY + "/datasets/EuRoCMAV/vicon_room1/V1_01_easy/V1_01_easy.slam" ,
   "VR12": SLAMBENCH2_DIRECTORY + "/datasets/EuRoCMAV/vicon_room1/V1_02_medium/V1_02_medium.slam" ,
   "VR13": SLAMBENCH2_DIRECTORY + "/datasets/EuRoCMAV/vicon_room1/V1_03_difficult/V1_03_difficult.slam" ,
   "VR21": SLAMBENCH2_DIRECTORY + "/datasets/EuRoCMAV/vicon_room2/V2_01_easy/V2_01_easy.slam" ,
   "VR22": SLAMBENCH2_DIRECTORY + "/datasets/EuRoCMAV/vicon_room2/V2_02_medium/V2_02_medium.slam" ,
   "VR23": SLAMBENCH2_DIRECTORY + "/datasets/EuRoCMAV/vicon_room2/V2_03_difficult/V2_03_difficult.slam" ,


            }
        
            
    }

    
    efusion_lowmem_arguments = {
        "*" : [  "--textureDim", "512",  "--nodeTextureDim", "512"]
    }
    
    efusion_medmem_arguments = {
        "*" : [  "--textureDim", "3072",  "--nodeTextureDim", "16384"]
    }

    efusion_highmem_arguments = {
        "*" : [  "--textureDim", "6144",  "--nodeTextureDim", "32768"]
    }



    orbslam2_mono_arguments = {
        "*" : [  "--mode" ,  "mono", "-voc", SLAMBENCH2_DIRECTORY + "/benchmarks/orbslam2/src/original/Vocabulary/ORBvoc.txt" ],        
    }

    orbslam2_stereo_arguments = {
        "*" : [ "--mode" ,  "stereo",  "-voc", SLAMBENCH2_DIRECTORY + "/benchmarks/orbslam2/src/original/Vocabulary/ORBvoc.txt" ],
    }
    
    orbslam2_rgbd_arguments = {
        "*" : [ "--mode" ,  "rgbd",  "-voc", SLAMBENCH2_DIRECTORY + "/benchmarks/orbslam2/src/original/Vocabulary/ORBvoc.txt" ],
    }

    kfusion_arguments = {
        "*" :                      [ "-z", "4", "-c", "2", "-r", "2" ],
        "-" :                      [  "-s", "8.0", "-d", "4,4,4",                      ],
        "living_room_traj0_loop" : [  "-s", "5.0", "-d", "1.7,2.5,1.2",                ],
        "living_room_traj1_loop" : [  "-s", "5.0", "-d", "2.425,2.5,2.75",             ],
        "living_room_traj2_loop" : [  "-s", "4.8", "-d", "1.6320001,2.400,1.1520001",  ],
        "living_room_traj3_loop" : [  "-s", "5.0", "-d", "1.3425,2.5,2",               ],
        "freiburg2_desk"         : [  "-s", "5.0", "-d", "0,1,3"   ,                   ]
    }


    libraries = {

        "RGBD" : {

        "kfusion-cpp"          : { "library_name" : SLAMBENCH2_DIRECTORY + "/build/lib/libkfusion-cpp-library.so" , "dataset_arguments" : kfusion_arguments } ,
        "kfusion-notoon"       : { "library_name" : SLAMBENCH2_DIRECTORY + "/build/lib/libkfusion-notoon-library.so" , "dataset_arguments" : kfusion_arguments } ,
        "kfusion-octree-cpp"   : { "library_name" : SLAMBENCH2_DIRECTORY + "/build/lib/libkfusion-octree-cpp-library.so" , "dataset_arguments" : kfusion_arguments } ,
        "kfusion-openmp"       : { "library_name" : SLAMBENCH2_DIRECTORY + "/build/lib/libkfusion-openmp-library.so", "dataset_arguments" : kfusion_arguments  },
        "kfusion-octree-openmp": { "library_name" : SLAMBENCH2_DIRECTORY + "/build/lib/libkfusion-octree-openmp-library.so", "dataset_arguments" : kfusion_arguments  },
        "kfusion-opencl"       : { "library_name" : SLAMBENCH2_DIRECTORY + "/build/lib/libkfusion-opencl-library.so", "dataset_arguments" : kfusion_arguments  },
        "kfusion-cuda"         : { "library_name" : SLAMBENCH2_DIRECTORY + "/build/lib/libkfusion-cuda-library.so", "dataset_arguments" : kfusion_arguments  },

        "efusion-cuda-hightmem"         : { "library_name" : SLAMBENCH2_DIRECTORY + "/build/lib/libefusion-cuda-library.so", "dataset_arguments" : efusion_lowmem_arguments  },
        "efusion-cuda-medmem"         : { "library_name" : SLAMBENCH2_DIRECTORY + "/build/lib/libefusion-cuda-library.so", "dataset_arguments" : efusion_medmem_arguments  },
        "efusion-cuda-lowmem"         : { "library_name" : SLAMBENCH2_DIRECTORY + "/build/lib/libefusion-cuda-library.so", "dataset_arguments" : efusion_highmem_arguments  },

        "infinitam-cuda"       : { "library_name" : SLAMBENCH2_DIRECTORY + "/build/lib/libinfinitam-cuda-library.so", "dataset_arguments" : {}  },
            
        "orbslam2-original-rgbd" : { "library_name" : SLAMBENCH2_DIRECTORY + "/build/lib/liborbslam2-original-library.so", "dataset_arguments" : orbslam2_rgbd_arguments   },

            } ,
        "MONO" : {
            
        "lsdslam-cpp"         : { "library_name" : SLAMBENCH2_DIRECTORY + "/build/lib/liblsdslam-cpp-library.so", "dataset_arguments" : {}  },
        "lsdslam-original_mp" : { "library_name" : SLAMBENCH2_DIRECTORY + "/build/lib/liblsdslam-original_mp-library.so", "dataset_arguments" : {}  },

        "monoslam-sequential" : { "library_name" : SLAMBENCH2_DIRECTORY + "/build/lib/libmonoslam-sequential-library.so", "dataset_arguments" : {}  },

        "orbslam2-original-mono" : { "library_name" : SLAMBENCH2_DIRECTORY + "/build/lib/liborbslam2-original-library.so", "dataset_arguments" : orbslam2_mono_arguments   },

        "ptam-original_mp" : { "library_name" : SLAMBENCH2_DIRECTORY + "/build/lib/libptam-original_mp-library.so", "dataset_arguments" : {}  },
        },

        "STEREO" : {
        
        "orbslam2-original-stereo" : { "library_name" : SLAMBENCH2_DIRECTORY + "/build/lib/liborbslam2-original-library.so", "dataset_arguments" : orbslam2_stereo_arguments   },
            
        "okvis-original" : { "library_name" : SLAMBENCH2_DIRECTORY + "/build/lib/libokvis-original-library.so", "dataset_arguments" : {}  },
        },
            
    }

    for mode in SKIP :
        if mode in libraries.keys() :
            del libraries[mode]
            
    for mode in libraries :
        for lib in SKIP :
            if lib in libraries[mode].keys() :
                del libraries[mode][lib]
        

    for implementation_categories in libraries.keys()  :
        for implementation in libraries[implementation_categories].keys()  :
            library = libraries[implementation_categories][implementation]
            if not os.path.exists(library["library_name"]) :
                print "Skip %s : file '%s' not found" % (implementation, library["library_name"])
                del libraries[implementation_categories][implementation]


    for dataset_categories in  datasets.keys() :
        for dataset_identifier in  datasets[dataset_categories].keys() :
                    
            dataset_filename = datasets[dataset_categories][dataset_identifier]                                        
            if not os.path.exists(dataset_filename) : 
                print "Skip %s : file not found" % (dataset_filename)
                del datasets[dataset_categories][dataset_identifier]


    testsuites = {}
    loader = SLAMBENCH2_DIRECTORY + "/build/bin/benchmark_loader"
    if not os.path.exists(loader) : 
        print "Skip all test: loader '%s' not found" % (loader)
        return testsuites
    

    for implementation_categories in libraries.keys()  :
        for implementation in libraries[implementation_categories].keys()  :
            testsuites[implementation] = []

            library = libraries[implementation_categories][implementation]
 
            for dataset_categories in  datasets.keys() :
                for dataset_identifier in  datasets[dataset_categories].keys() :
                    
                    dataset_filename = datasets[dataset_categories][dataset_identifier]                                        

            
                    overlap = set (implementation_categories.split(",")) &  set (dataset_categories.split(","))                    
                    if len(overlap) == 0 :
                        print "Skip the couple %s - %s: overlap" % (implementation, dataset_identifier)
                        continue


                    
                    
                    command  = [ loader ]
                    command += [ "-i",  dataset_filename ]
                    command += [ "-load", library["library_name"]]
                    command += [ "-o" , OUTPUT_LOG_DIRECTORY + "/benchmark-%s-test-%s.log" % (implementation,dataset_identifier)]
                    
                    if "*" in library["dataset_arguments"] :
                        command += library["dataset_arguments"]["*"]
                        
                    if dataset_identifier in library["dataset_arguments"] :
                        command += library["dataset_arguments"][dataset_identifier]
                    elif "-" in library["dataset_arguments"] :
                        command += library["dataset_arguments"]["-"]
                        
                    testsuites[implementation].append ({ "classname" : "benchmark-%s" % implementation,  
                                                         "name"      : "test-%s-execution" % dataset_identifier , 
                                                         "command"   : command ,
                                                         "env"       : env_variables })

    return testsuites




################# SETUP FILES  #######################





################# MAIN   #######################





def main():

    #### VARIABLES  ####
    
    OUTPUT_FILE = None   
    CHECK_ONLY  = False
    SLAMBENCH2_DIRECTORY= "./repository"
    OUTPUT_LOG_DIRECTORY= "./log"
    SKIP = []

    

    #### CHECK ENV  ####


    for x in ["JOB_NAME", "WORKSPACE", "JENKINS_TOOLS"] :
        if not x in  os.environ :
            printerr("%s is not defined" % x)
            exit(1)


    if os.getcwd() != os.environ["WORKSPACE"] :
        printerr( "pwd = '%s' is different to the WORKSPACE = '%s'." % (os.getcwd(), os.environ["WORKSPACE"]))
        exit(1)

    if "SKIP" in  os.environ :
        SKIP = os.environ["SKIP"].split(",")

    
    #### PARSE ARGUMENTS  ####
    
    parser = argparse.ArgumentParser()
    parser.add_argument("-l","--logdir",       help="Log directory.", type=str, required=True)
    parser.add_argument("-s","--slambench",    help="SLAMBench2 directory.", type=str, required=True)
    parser.add_argument("-o","--output",       help="JUnit report output file.", type=str)
    parser.add_argument("-c","--checkonly",    help="Skip the command execution pass.",  action="store_true")
    parser.add_argument("-v", "--verbose",     help="Increase output verbosity.", type=int, choices=[0, 1, 2])
    args = parser.parse_args()
    
    printerr( "Check only : %s" % args.checkonly)
    printerr( "Output file: "+  args.output)
    printerr( "Log directory : "+ args.logdir)
    printerr( "SLAMBench2 directory: "+ args.slambench)
    
    CHECK_ONLY  = args.checkonly
    OUTPUT_FILE = args.output
    SLAMBENCH2_DIRECTORY= args.slambench
    OUTPUT_LOG_DIRECTORY= args.logdir


    #### RUN PREPARATION COMMANDS  ####

    preparation_commands = ["mkdir -p " + OUTPUT_LOG_DIRECTORY]

    for command in preparation_commands :
        if not CHECK_ONLY :
            exitstatus = os.system(command)
        else :
            printerr ( "prepare_command: %s" % command)
            exitstatus = 0
        if exitstatus :
            printerr ( "command %s return %d" % (command,exitstatus))
            exit(1)

            

    #### RUN TEST SUITE  ####
    
    testsuites = generate_test_units(OUTPUT_LOG_DIRECTORY, SKIP, SLAMBENCH2_DIRECTORY)         

    x = testsuites.keys()
    shuffle(x)
    
    run_results = []
    
    printerr( "Number of testsuites to be done : %d  " % (len(x)))

    testsuite_count = 0
    for testsuite_name in x :
        testsuite_count += 1
        testsuite = testsuites[testsuite_name]
        printerr( "Start test suite %s %d/%d." % (testsuite_name,testsuite_count,len(x)))
        test_count = 0
        for test_to_run in testsuite :
            test_count += 1
            printerr( "Run test suite %s (%d/%d) - test %s (%d/%d)." % (testsuite_name,testsuite_count,len(x),test_to_run["name"],test_count,len(testsuite)))
            run_results +=  [jenkinstest.run_test (test_to_run , logdir = OUTPUT_LOG_DIRECTORY, print_only = CHECK_ONLY )]

           

    #### WRITE TEST SUITE RESULTS  ####
    
    if OUTPUT_FILE != None :
        printerr( "Write output file : " + OUTPUT_FILE)
        raw_xml = jenkinstest.print_testsuite (testsuite_name,run_results)
        f = open(OUTPUT_FILE, 'w')
        f.write(raw_xml)
        f.close()

    #### END  ####

    printerr("End of program.")
    exit(0)


main()
