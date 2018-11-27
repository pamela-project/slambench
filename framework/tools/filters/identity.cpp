/*
 * identity.cpp
 *
 *  Created on: Nov 22, 2018
 *      Author: toky
 */

#include <SLAMBenchAPI.h>
#include <io/IdentityFrame.h>
#include <io/sensor/CameraSensorFinder.h>


bool sb_new_filter_configuration (SLAMBenchFilterLibraryHelper * filter_settings) {
	return true;
}

bool sb_init_filter (SLAMBenchFilterLibraryHelper * filter_settings) {
	return true;
}


 slambench::io::SLAMFrame * sb_process_filter (SLAMBenchFilterLibraryHelper * , slambench::io::SLAMFrame * frame) {
	return new IdentityFrame(frame);
}


