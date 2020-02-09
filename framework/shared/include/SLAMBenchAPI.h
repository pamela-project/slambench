/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef FRAMEWORK_SHARED_INCLUDE_SLAMBENCHAPI_H_
#define FRAMEWORK_SHARED_INCLUDE_SLAMBENCHAPI_H_

#include <SLAMBenchLibraryHelper.h>
#include <SLAMBenchConfigurationLifelong.h>

#include <Eigen/Core>


namespace slambench {
	namespace io {
		class SLAMFrame;
	}
}

/*
 * Those functions define SLAMBenchConfiguration and reuse it after parameters parsing
 * sb_new_slam_configuration: allocate slam_settings
 * sb_init_slam_system: retrieve arguments from slam_settings
 * */

bool sb_new_slam_configuration(SLAMBenchLibraryHelper * slam_settings) ;
bool sb_init_slam_system(SLAMBenchLibraryHelper * slam_settings) ;


/*
 * Every frame we update the sensors and we process
 * sb_update_frame: update information for a specific sensor, must be call for each sensor
 * sb_process_once: once every sensor are updated slam process them
 * */

bool sb_update_frame (SLAMBenchLibraryHelper * slam_settings, slambench::io::SLAMFrame * type) ;
bool sb_relocalize (SLAMBenchLibraryHelper * slam_settings) ;
bool sb_process_once (SLAMBenchLibraryHelper * slam_settings) ;


/*
 * At any time, the SLAM system needs to provide its outputs
 * */

bool sb_update_outputs(SLAMBenchLibraryHelper *lib, const slambench::TimeStamp *latest_output);

/*
 * At the end of the process the SLAM system can be ask to save the volume in a file and to clean memory
 * */
bool sb_clean_slam_system();

/*
 * C equivalent of each function (not used)
 * */



extern "C" {
bool c_sb_new_slam_configuration(void * slam_settings) ;
bool c_sb_init_slam_system(void* slam_settings) ;
bool c_sb_update_frame (void * slam_settings, void * type) ;
bool c_sb_relocalize (void * slam_settings) ;
bool c_sb_process_once (void * slam_settings) ;
bool c_sb_update_outputs(void *lib, void *timestamp);
bool c_sb_clean_slam_system();

}

#endif /* FRAMEWORK_SHARED_INCLUDE_SLAMBENCHAPI_H_ */
