/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

 
#include <SLAMBenchAPI.h>
#include <TimeStamp.h>
#include <Eigen/Core>

extern "C" {

	bool c_sb_new_slam_configuration(void * slam_settings)  {
		return sb_new_slam_configuration((SLAMBenchLibraryHelper *) slam_settings);
	}

	bool c_sb_init_slam_system(void * slam_settings)  {
		return sb_init_slam_system((SLAMBenchLibraryHelper *) slam_settings);
	}
	
	bool c_sb_update_frame(void * slam_settings, void * type)  {
		return sb_update_frame((SLAMBenchLibraryHelper *) slam_settings, (slambench::io::SLAMFrame  *) type);
	}

	bool c_sb_relocalize(void * slam_settings)  {
		return sb_relocalize((SLAMBenchLibraryHelper *) slam_settings);
	}

	bool c_sb_process_once (void * slam_settings){
		return sb_process_once((SLAMBenchLibraryHelper *) slam_settings);
	}

	bool c_sb_clean_slam_system(){
		return sb_clean_slam_system();
	}
	
	bool c_sb_update_outputs(void *lib, void *timestamp) {
		return sb_update_outputs((SLAMBenchLibraryHelper*)lib, (const slambench::TimeStamp *)timestamp);
	}
}
