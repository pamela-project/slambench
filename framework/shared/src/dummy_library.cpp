/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include <SLAMBenchAPI.h>

bool sb_new_slam_configuration(SLAMBenchLibraryHelper * slam_settings) {
  return false;
}

bool sb_init_slam_system(SLAMBenchLibraryHelper * slam_settings)  {
  return false;

}

bool sb_update_frame (SLAMBenchLibraryHelper * , slambench::io::SLAMFrame* s) {
  return false;
}

bool sb_process_once (SLAMBenchLibraryHelper * slam_settings)  {
  return false;
}

bool sb_clean_slam_system() {
    return false;
}

bool sb_update_outputs(SLAMBenchLibraryHelper *lib, const slambench::TimeStamp *timestamp) {
	return false;
}

