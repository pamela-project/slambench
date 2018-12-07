/*
 * skip_frame.cpp
 *
 *  Created on: Nov 22, 2018
 *      Author: toky
 */

#include <SLAMBenchAPI.h>
#include <io/IdentityFrame.h>
#include <io/sensor/DepthSensor.h>
#include <io/sensor/CameraSensorFinder.h>
#include <io/FrameBuffer.h>

double threshold = 0;
double default_threshold = 0.2;
static slambench::io::DepthSensor *depth_sensor;
std::map<slambench::io::Sensor*,slambench::io::SLAMFrame *> last_frames;

bool compare_depth_images (short * previous , short * current) {
	assert(previous != current);
	return false;
}

bool compare (slambench::io::SLAMFrame * previous , slambench::io::SLAMFrame * current) {

	// This buffer is there to fix a design problem in SLAMBench, this will be addressed in the future
	static slambench::io::FrameBuffer workaround_for_GetData;

	assert(previous->FrameSensor == current->FrameSensor);
	assert(previous != current);

	if (current->FrameSensor == depth_sensor) {

		workaround_for_GetData.Acquire();
		workaround_for_GetData.Reserve(previous->GetSize());
		memcpy(workaround_for_GetData.Data(),previous->GetData(),previous->GetSize());
		previous->FreeData();

		bool res =  compare_depth_images((short *) workaround_for_GetData.Data(), (short *) current->GetData());
		current->FreeData();
		workaround_for_GetData.Release();
		return res;
	} else {
		return false;
	}


}


bool sb_new_filter_configuration (SLAMBenchFilterLibraryHelper * filter_settings) {
	filter_settings->addParameter(TypedParameter<double>("fth", "skip-threshold",     "Numerical value to specify when the threshold is reached.",    &threshold, &default_threshold));
	return true;
}

bool sb_init_filter (SLAMBenchFilterLibraryHelper * filter_settings) {


	slambench::io::CameraSensorFinder sensor_finder;
	depth_sensor = (slambench::io::DepthSensor*)sensor_finder.FindOne(filter_settings->get_sensors(), {{"camera_type", "depth"}});
	assert(depth_sensor);
	return true;
}

 slambench::io::SLAMFrame * sb_process_filter (SLAMBenchFilterLibraryHelper * , slambench::io::SLAMFrame * frame) {

	 slambench::io::SLAMFrame * new_frame = nullptr;
	 if (last_frames.find(frame->FrameSensor) != last_frames.end()) {

		 // previous frame found.
		 if (compare (frame, last_frames.at(frame->FrameSensor))) {
			 std::cout << "** Skip one frame." << std::endl;
		 } else {
			 new_frame = new IdentityFrame(frame);
		 }

	 } else {
		 new_frame = new IdentityFrame(frame);
	 }

	 last_frames.insert(std::pair<slambench::io::Sensor*,slambench::io::SLAMFrame *>(frame->FrameSensor,frame));

	 return new_frame;

}






